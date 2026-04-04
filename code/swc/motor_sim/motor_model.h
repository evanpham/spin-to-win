/**
 * @file motor_model.h
 * @brief Physics-based motor model for firmware unit testing and control loop validation.
 *
 * Models a 3-phase motor in the natural (abc) frame. Per-phase voltages are the primary
 * input; per-phase currents and rotor state are the primary outputs. Clarke/Park transforms
 * are not applied internally — the model operates on physical phase quantities, keeping
 * the boundary between physical simulation and frame-transformation math explicit.
 *
 * Amplitude-invariant convention is used throughout. Phase current magnitudes in the abc
 * frame are preserved under Clarke transform — i_alpha peaks at the same value as i_a.
 *
 * Intended use: link motor_model.c into host-machine test binaries alongside firmware SWCs.
 * The model is not designed for target deployment; floating-point and dynamic allocation
 * assumptions may not hold on all embedded targets.
 *
 * Error handling convention:
 *   - Configuration functions (init, set_*) return MotorModelError codes.
 *     Callers should check these; a non-zero return indicates the model state was not
 *     modified.
 *   - motor_model_step() asserts on NULL or uninitialized model pointer — these
 *     represent programmer error in a test harness context and should crash loudly.
 *     Runtime simulation faults (e.g. numerical divergence) set model->error and can
 *     be checked with motor_model_get_error() at whatever cadence suits the test harness.
 */

#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * Constants
 * ───────────────────────────────────────────────────────────────────────────── */

/** Number of phases. Fixed at 3; present as a named constant to avoid magic numbers
 *  in loop bounds throughout the implementation. */
#define MOTOR_MODEL_NUM_PHASES 3

/* ─────────────────────────────────────────────────────────────────────────────
 * Error codes
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Return type for all configuration-time functions.
 *
 * Runtime faults during step() are reported via the model's internal error field
 * rather than as return values. See motor_model_get_error().
 */
typedef enum {
    MOTOR_MODEL_OK               = 0,  /**< Operation succeeded. Model state was modified
                                            as described by the called function. */
    MOTOR_MODEL_ERR_NULL_PTR     = 1,  /**< A required pointer argument was NULL.
                                            Model state not modified. */
    MOTOR_MODEL_ERR_INVALID_PARAM = 2, /**< A parameter value was out of physical range
                                            (e.g. negative inductance, zero dt, pole count
                                            that is not a positive even integer).
                                            Model state not modified. */
    MOTOR_MODEL_ERR_NOT_INIT     = 3,  /**< Operation requires the model to have been
                                            successfully initialized via motor_model_init().
                                            Model state not modified. */
    MOTOR_MODEL_ERR_RUNTIME      = 4,  /**< A runtime fault was detected during step().
                                            Stored in model->error; not returned directly
                                            by step(). Retrieved via motor_model_get_error(). */
} MotorModelError;

/* ─────────────────────────────────────────────────────────────────────────────
 * Fault flags
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Per-phase fault injection flags.
 *
 * Faults are injected via motor_model_set_phase_fault() and cleared via
 * motor_model_clear_phase_faults(). Multiple fault types can be active simultaneously
 * on the same phase; behavior under conflicting faults (open + short) is
 * implementation-defined and should not be relied upon.
 */
typedef struct {
    bool open[MOTOR_MODEL_NUM_PHASES];   /**< If true for phase N, that phase is treated as
                                              disconnected. Terminal voltage has no effect;
                                              phase current is forced to zero and decays
                                              naturally through back-EMF. */
    bool shorted[MOTOR_MODEL_NUM_PHASES];/**< If true for phase N, that phase terminal is
                                              shorted to the negative DC bus. Phase voltage
                                              is overridden to 0V; current is limited only
                                              by phase resistance and inductance. */
} MotorFaultFlags;

/* ─────────────────────────────────────────────────────────────────────────────
 * Parameter struct
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Motor physical parameters.
 *
 * Required fields: L, R, J, Ke.
 * Optional fields have documented defaults applied by motor_model_init() when
 * the corresponding flag is not set. Defaults represent ideal/simplified behavior.
 *
 * Unit system: SI throughout (H, Ω, kg·m², V·s/rad, N·m·s/rad).
 * Motor constants Ke, Kt, and Kv are related by:
 *   Ke = Kt = 1/Kv  (in SI units, with amplitude-invariant Clarke convention)
 * Only Ke needs to be provided; Kt is derived internally.
 */
typedef struct {
    /* ── Required ── */
    float L;          /**< Phase inductance [H]. Must be > 0. Assumed symmetric across
                           phases. Per-phase inductance variation not yet modeled. */
    float R;          /**< Phase resistance [Ω]. Must be > 0. Assumed symmetric across
                           phases. Temperature dependence not yet modeled. */
    float J;          /**< Rotor moment of inertia [kg·m²]. Must be > 0. Includes rotor
                           only; load inertia is applied separately via set_load_torque(). */
    float Ke;         /**< Back-EMF constant [V·s/rad], defined in electrical radians.
                           Must be > 0. Kt = Ke in SI with amplitude-invariant convention. */

    /* ── Optional — mechanical ── */
    float drag;       /**< Viscous drag coefficient [N·m·s/rad]. Velocity-proportional
                           loss term: T_drag = drag * omega_mechanical. Default: 0.0
                           (ideal, no friction). Must be >= 0. */

    /* ── Optional — geometry ── */
    int   poles;      /**< Total pole count (not pole pairs). Must be a positive even
                           integer if provided. Used to convert between electrical and
                           mechanical angle: theta_e = (poles/2) * theta_m.
                           Default: 2 (one pole pair; electrical angle == mechanical angle). */
    int   slots;      /**< Stator slot count. Not used in current model equations;
                           reserved for future cogging torque and inductance ripple modeling.
                           Default: 0 (ignored). */
} MotorParams;

/* ─────────────────────────────────────────────────────────────────────────────
 * Model state struct
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Complete motor model state.
 *
 * Treat as opaque from test code where possible; access state through getter functions
 * rather than reading fields directly. Direct field access is permitted for debugging
 * but creates coupling to the internal representation.
 *
 * All angle quantities are in radians. Electrical angle wraps on [0, 2π).
 * Angular velocity is in rad/s (electrical).
 */
typedef struct {
    /* ── Parameters (owned copy) ── */
    MotorParams params;              /**< Physical parameters. Modified only via
                                         motor_model_set_*() functions after init. */
    float dt;                        /**< Integration timestep [s]. Set at init and
                                         modifiable via motor_model_set_dt(). */

    /* ── Electrical state ── */
    float i_phase[MOTOR_MODEL_NUM_PHASES]; /**< Per-phase currents [A], abc frame.
                                                Positive convention: current flowing
                                                into the motor terminal. */
    float v_phase[MOTOR_MODEL_NUM_PHASES]; /**< Per-phase terminal voltages [V] applied
                                                at last step(). Set by
                                                motor_model_set_phase_voltages(). */

    /* ── Mechanical state ── */
    float theta_e;    /**< Electrical angle [rad], range [0, 2π). Derived from mechanical
                           angle via theta_e = (poles/2) * theta_m, then wrapped. */
    float theta_m;    /**< Mechanical angle [rad], unbounded (does not wrap). Useful for
                           multi-turn position tracking in test scenarios. */
    float omega_e;    /**< Electrical angular velocity [rad/s]. Positive = forward rotation
                           as defined by phase sequence abc. */
    float omega_m;    /**< Mechanical angular velocity [rad/s]. omega_m = omega_e / (poles/2). */

    /* ── Load ── */
    float T_load;     /**< External load torque [N·m] applied to rotor shaft. Positive
                           value opposes forward rotation (resistive load convention).
                           Set via motor_model_set_load_torque(). Default: 0.0. */

    /* ── Fault state ── */
    MotorFaultFlags faults;          /**< Active fault injections. Modified via
                                         motor_model_set_phase_fault() and
                                         motor_model_clear_phase_faults(). */

    /* ── Runtime error ── */
    MotorModelError error;           /**< Set by step() if a runtime fault is detected
                                         (e.g. current divergence suggesting numerical
                                         instability). Cleared by motor_model_clear_error()
                                         or motor_model_reset(). */

    /* ── Lifecycle ── */
    bool initialized;                /**< Set true by a successful motor_model_init() call.
                                         Guards against use before initialization. */
} MotorModel;

/* ─────────────────────────────────────────────────────────────────────────────
 * Lifecycle
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize a MotorModel instance with the given parameters and timestep.
 *
 * Validates all required parameters, applies defaults for optional fields, and sets
 * all state variables to zero (rotor at rest, zero current, zero angle). After a
 * successful call, model->initialized is true and the model is ready for use.
 *
 * @param[out] model  Pointer to an allocated MotorModel struct. All fields are
 *                    overwritten on success. Must not be NULL.
 * @param[in]  params Motor physical parameters. L, R, J, and Ke must be > 0.
 *                    poles must be a positive even integer or 0 (default applied).
 *                    drag must be >= 0.
 * @param[in]  dt     Integration timestep [s]. Must be > 0. Typically set to match
 *                    the FOC loop period (e.g. 1.0f/18500.0f for 18.5 kHz).
 *                    Numerical stability requires dt << L/R (electrical time constant).
 *
 * @return MOTOR_MODEL_OK             on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR   if model is NULL.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if any required parameter fails validation,
 *                                    or if dt <= 0. model->initialized remains false.
 */
MotorModelError motor_model_init(MotorModel *model, MotorParams params, float dt);

/**
 * @brief Reset all dynamic state to initial conditions without changing parameters.
 *
 * Clears phase currents, voltages, rotor angle, angular velocity, load torque,
 * fault flags, and runtime error. Parameters (L, R, J, Ke, dt, etc.) are preserved.
 * Useful for running multiple test cases sequentially without re-initializing.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->initialized
 *                       remains true after reset. Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_reset(MotorModel *model);

/* ─────────────────────────────────────────────────────────────────────────────
 * Configuration setters
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Set the integration timestep.
 *
 * Takes effect on the next call to motor_model_step(). Does not reset model state.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->dt is updated
 *                       on success. Must not be NULL.
 * @param[in]     dt     New timestep [s]. Must be > 0.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if dt <= 0. model->dt is not modified.
 */
MotorModelError motor_model_set_dt(MotorModel *model, float dt);

/**
 * @brief Set phase inductance.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->params.L is
 *                       updated on success. Must not be NULL.
 * @param[in]     L      New phase inductance [H]. Must be > 0.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if L <= 0. model->params.L is not modified.
 */
MotorModelError motor_model_set_L(MotorModel *model, float L);

/**
 * @brief Set phase resistance.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->params.R is
 *                       updated on success. Must not be NULL.
 * @param[in]     R      New phase resistance [Ω]. Must be > 0.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if R <= 0. model->params.R is not modified.
 */
MotorModelError motor_model_set_R(MotorModel *model, float R);

/**
 * @brief Set rotor moment of inertia.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->params.J is
 *                       updated on success. Must not be NULL.
 * @param[in]     J      New moment of inertia [kg·m²]. Must be > 0.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if J <= 0. model->params.J is not modified.
 */
MotorModelError motor_model_set_J(MotorModel *model, float J);

/**
 * @brief Set back-EMF constant.
 *
 * Ke, Kt, and Kv are related: Ke = Kt = 1/Kv in SI with amplitude-invariant Clarke.
 * Only Ke is stored; derive the others at the call site if needed.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->params.Ke is
 *                       updated on success. Must not be NULL.
 * @param[in]     Ke     New back-EMF constant [V·s/rad electrical]. Must be > 0.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if Ke <= 0. model->params.Ke is not modified.
 */
MotorModelError motor_model_set_Ke(MotorModel *model, float Ke);

/**
 * @brief Set viscous drag coefficient.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->params.drag is
 *                       updated on success. Must not be NULL.
 * @param[in]     drag   New drag coefficient [N·m·s/rad]. Must be >= 0.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if drag < 0. model->params.drag not modified.
 */
MotorModelError motor_model_set_drag(MotorModel *model, float drag);

/**
 * @brief Set the external mechanical load torque.
 *
 * Models a torque applied to the rotor shaft opposing forward rotation. Use this to
 * simulate step loads, ramp loads, or speed-dependent load curves in test scenarios.
 * For a speed-dependent load, call this function each step with a value derived from
 * the current omega_m returned by motor_model_get_rotor_velocity().
 *
 * @param[in,out] model   Pointer to an initialized MotorModel. model->T_load is
 *                        updated on success. Must not be NULL.
 * @param[in]     T_load  Load torque [N·m]. Positive value opposes forward rotation.
 *                        Negative value assists rotation (e.g. overhauling load).
 *                        No range restriction; sign convention is caller's responsibility.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_set_load_torque(MotorModel *model, float T_load);

/**
 * @brief Inject a fault condition on a specific phase.
 *
 * Faults take effect on the next call to motor_model_step(). Multiple faults may be
 * active simultaneously. Behavior when both open and shorted are set on the same
 * phase is implementation-defined.
 *
 * @param[in,out] model    Pointer to an initialized MotorModel. The relevant field in
 *                         model->faults is set to true on success. Must not be NULL.
 * @param[in]     phase    Phase index: 0 = phase A, 1 = phase B, 2 = phase C.
 *                         Must be in [0, MOTOR_MODEL_NUM_PHASES).
 * @param[in]     open     If true, set this phase as open-circuited.
 * @param[in]     shorted  If true, set this phase as shorted to negative DC bus.
 *
 * @return MOTOR_MODEL_OK              on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR    if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT    if model has not been successfully initialized.
 * @return MOTOR_MODEL_ERR_INVALID_PARAM if phase is out of range. Fault flags not modified.
 */
MotorModelError motor_model_set_phase_fault(MotorModel *model, int phase,
                                             bool open, bool shorted);

/**
 * @brief Clear all active fault injections on all phases.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. All fields in
 *                       model->faults are set to false on success. Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_clear_phase_faults(MotorModel *model);

/* ─────────────────────────────────────────────────────────────────────────────
 * Simulation interface
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Set the phase terminal voltages to be applied on the next step().
 *
 * Voltages are latched into model->v_phase and applied during the next integration
 * step. Call this once per control cycle before motor_model_step(), mirroring the
 * pattern in firmware where SVM output is written to the PWM peripheral and takes
 * effect on the next period.
 *
 * @param[in,out] model   Pointer to an initialized MotorModel. model->v_phase[] is
 *                        updated on success. Must not be NULL.
 * @param[in]     v_a     Phase A terminal voltage [V] relative to DC bus negative.
 * @param[in]     v_b     Phase B terminal voltage [V] relative to DC bus negative.
 * @param[in]     v_c     Phase C terminal voltage [V] relative to DC bus negative.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_set_phase_voltages(MotorModel *model,
                                                float v_a, float v_b, float v_c);

/**
 * @brief Advance the motor model by one timestep dt.
 *
 * Integrates the electrical and mechanical equations of motion using the currently
 * latched phase voltages (set via motor_model_set_phase_voltages()), active fault
 * flags, and current load torque. Updates all state fields in model:
 *   - model->i_phase[]  (forward Euler integration of electrical dynamics)
 *   - model->theta_e    (wrapped to [0, 2π))
 *   - model->theta_m    (unbounded)
 *   - model->omega_e
 *   - model->omega_m
 *
 * If a runtime fault is detected (e.g. current magnitude exceeds a sanity threshold
 * suggesting numerical divergence), model->error is set to MOTOR_MODEL_ERR_RUNTIME
 * and the state at that step is preserved as-is for inspection. Subsequent calls to
 * step() while model->error is set are no-ops; call motor_model_clear_error() to
 * resume simulation.
 *
 * Returns void — check model->error via motor_model_get_error() as needed.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. Must not be NULL and
 *                       must have been successfully initialized. Asserts on NULL or
 *                       uninitialized model — these are programmer errors in a test
 *                       harness context and should crash loudly rather than produce
 *                       silently wrong results.
 */
void motor_model_step(MotorModel *model);

/* ─────────────────────────────────────────────────────────────────────────────
 * State getters
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Get the current electrical angle of the rotor.
 *
 * @param[in]  model      Pointer to an initialized MotorModel. Must not be NULL.
 * @param[out] theta_e    Set to model->theta_e [rad], range [0, 2π). Not modified
 *                        on error. Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model or theta_e is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_get_angle(const MotorModel *model, float *theta_e);

/**
 * @brief Get the current phase currents in the abc frame.
 *
 * @param[in]  model    Pointer to an initialized MotorModel. Must not be NULL.
 * @param[out] i_a      Set to model->i_phase[0] [A]. Not modified on error. Must not
 *                      be NULL.
 * @param[out] i_b      Set to model->i_phase[1] [A]. Not modified on error. Must not
 *                      be NULL.
 * @param[out] i_c      Set to model->i_phase[2] [A]. Not modified on error. Must not
 *                      be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model, i_a, i_b, or i_c is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_get_phase_currents(const MotorModel *model,
                                                float *i_a, float *i_b, float *i_c);

/**
 * @brief Get the current rotor angular velocity.
 *
 * @param[in]  model    Pointer to an initialized MotorModel. Must not be NULL.
 * @param[out] omega_m  Set to model->omega_m [rad/s mechanical]. Not modified on
 *                      error. Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model or omega_m is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_get_rotor_velocity(const MotorModel *model, float *omega_m);

/**
 * @brief Get the instantaneous electromagnetic torque.
 *
 * Computed from current phase currents and rotor angle as:
 *   T_em = Kt * (i_a * sin(theta_e) + i_b * sin(theta_e - 2π/3) + i_c * sin(theta_e + 2π/3))
 * where Kt = Ke (amplitude-invariant convention).
 *
 * @param[in]  model   Pointer to an initialized MotorModel. Must not be NULL.
 * @param[out] torque  Set to the computed electromagnetic torque [N·m]. Positive
 *                     value drives forward rotation. Not modified on error.
 *                     Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model or torque is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_get_torque(const MotorModel *model, float *torque);

/**
 * @brief Get the instantaneous per-phase back-EMF voltages.
 *
 * Computed as:
 *   e_a = Ke * omega_e * sin(theta_e)
 *   e_b = Ke * omega_e * sin(theta_e - 2π/3)
 *   e_c = Ke * omega_e * sin(theta_e + 2π/3)
 *
 * Useful for validating observer BEMF estimation against ground truth.
 *
 * @param[in]  model  Pointer to an initialized MotorModel. Must not be NULL.
 * @param[out] e_a    Set to back-EMF of phase A [V]. Not modified on error.
 *                    Must not be NULL.
 * @param[out] e_b    Set to back-EMF of phase B [V]. Not modified on error.
 *                    Must not be NULL.
 * @param[out] e_c    Set to back-EMF of phase C [V]. Not modified on error.
 *                    Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model, e_a, e_b, or e_c is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_get_back_emf(const MotorModel *model,
                                          float *e_a, float *e_b, float *e_c);

/* ─────────────────────────────────────────────────────────────────────────────
 * Error handling
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Retrieve the current runtime error state.
 *
 * @param[in] model  Pointer to an initialized MotorModel. Must not be NULL.
 *
 * @return The current value of model->error. Returns MOTOR_MODEL_ERR_NULL_PTR if
 *         model is NULL, MOTOR_MODEL_ERR_NOT_INIT if not initialized, or
 *         MOTOR_MODEL_OK if no runtime fault is active.
 */
MotorModelError motor_model_get_error(const MotorModel *model);

/**
 * @brief Clear the runtime error state, allowing step() to resume.
 *
 * Does not reset model state; use motor_model_reset() if a clean state is also needed.
 *
 * @param[in,out] model  Pointer to an initialized MotorModel. model->error is set to
 *                       MOTOR_MODEL_OK on success. Must not be NULL.
 *
 * @return MOTOR_MODEL_OK           on success.
 * @return MOTOR_MODEL_ERR_NULL_PTR if model is NULL.
 * @return MOTOR_MODEL_ERR_NOT_INIT if model has not been successfully initialized.
 */
MotorModelError motor_model_clear_error(MotorModel *model);

#endif /* MOTOR_MODEL_H */

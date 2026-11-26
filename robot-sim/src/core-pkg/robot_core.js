/**
 * Body levelling: PI on the IMU attitude error, D on the gyro rate, slew and
 * range limited. Same law as `hk::Balance` in the firmware (CONTROL.md §7).
 */
export class Balance {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        BalanceFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_balance_free(ptr, 0);
    }
    /**
     * Gains as in hexapod.config.js `control.balance`; angles in degrees.
     * @param {number} kp
     * @param {number} ki
     * @param {number} kd
     * @param {number} i_max_deg
     * @param {number} slew_deg_s
     * @param {number} d_lowpass_hz
     * @param {number} limit_deg
     */
    constructor(kp, ki, kd, i_max_deg, slew_deg_s, d_lowpass_hz, limit_deg) {
        const ret = wasm.balance_new(kp, ki, kd, i_max_deg, slew_deg_s, d_lowpass_hz, limit_deg);
        this.__wbg_ptr = ret;
        BalanceFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {number}
     */
    pitch() {
        const ret = wasm.balance_pitch(this.__wbg_ptr);
        return ret;
    }
    reset() {
        wasm.balance_reset(this.__wbg_ptr);
    }
    /**
     * @returns {number}
     */
    roll() {
        const ret = wasm.balance_roll(this.__wbg_ptr);
        return ret;
    }
    /**
     * target / measured roll and pitch (rad), gyro rates (rad/s).
     * @param {number} dt
     * @param {number} t_roll
     * @param {number} t_pitch
     * @param {number} m_roll
     * @param {number} m_pitch
     * @param {number} r_roll
     * @param {number} r_pitch
     */
    update(dt, t_roll, t_pitch, m_roll, m_pitch, r_roll, r_pitch) {
        wasm.balance_update(this.__wbg_ptr, dt, t_roll, t_pitch, m_roll, m_pitch, r_roll, r_pitch);
    }
}
if (Symbol.dispose) Balance.prototype[Symbol.dispose] = Balance.prototype.free;

export class HexapodGait {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        HexapodGaitFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_hexapodgait_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    activity() {
        const ret = wasm.hexapodgait_activity(this.__wbg_ptr);
        return ret;
    }
    /**
     * [coxa, femur, tibia] * 6, radians.
     * @returns {Float32Array}
     */
    angles() {
        const ret = wasm.hexapodgait_angles(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Body bob (m), unused while the body follows the feet.
     * @returns {number}
     */
    bob() {
        const ret = wasm.hexapodgait_bob(this.__wbg_ptr);
        return ret;
    }
    /**
     * Number of legs whose last target was clamped.
     * @returns {number}
     */
    clamped_legs() {
        const ret = wasm.hexapodgait_clamped_legs(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Bit i = leg i's last target was out of reach or hit a joint limit.
     * @returns {number}
     */
    clamped_mask() {
        const ret = wasm.hexapodgait_clamped_mask(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * The gait actually running (may lag `set_gait` until a phase boundary).
     * @returns {number}
     */
    current_gait() {
        const ret = wasm.hexapodgait_current_gait(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Foot targets in the level frame, [x, y, z] * 6.
     * @returns {Float32Array}
     */
    feet() {
        const ret = wasm.hexapodgait_feet(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Foot targets in the posed body frame, [x, y, z] * 6 (after the lift cap).
     * @returns {Float32Array}
     */
    feet_body() {
        const ret = wasm.hexapodgait_feet_body(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Per-leg ground offsets learned at touchdown (m, + = higher ground).
     * @returns {Float32Array}
     */
    ground_offsets() {
        const ret = wasm.hexapodgait_ground_offsets(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * `mounts` = [x, z, yaw] * 6 for the coxa joints in the body frame.
     * `foot_reach` = horizontal distance from the coxa joint to the neutral foot.
     * @param {Float32Array | number[]} mounts
     * @param {number} coxa
     * @param {number} femur
     * @param {number} tibia
     * @param {number} foot_reach
     * @param {number} body_height
     */
    constructor(mounts, coxa, femur, tibia, foot_reach, body_height) {
        const val2 = +coxa;
        const val3 = +femur;
        const val4 = +tibia;
        const val5 = +foot_reach;
        const val6 = +body_height;
        const ptr0 = passArrayF32ToWasm0(mounts, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.hexapodgait_new(ptr0, len0, val2, val3, val4, val5, val6);
        this.__wbg_ptr = ret;
        HexapodGaitFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {number}
     */
    phase() {
        const ret = wasm.hexapodgait_phase(this.__wbg_ptr);
        return ret;
    }
    /**
     * Measured body roll / pitch (IMU, rad). Touchdown heights are then taken in
     * the true level frame: without it, a body that is not where it was told to
     * be learns its own tilt into the ground offsets (front feet "find" higher
     * ground, rear feet search lower) and the tilt grows.
     * @param {number} roll
     * @param {number} pitch
     */
    set_attitude(roll, pitch) {
        wasm.hexapodgait_set_attitude(this.__wbg_ptr, roll, pitch);
    }
    /**
     * @param {number} h
     */
    set_body_height(h) {
        wasm.hexapodgait_set_body_height(this.__wbg_ptr, h);
    }
    /**
     * Foot switches, bit i = leg i touching the ground.
     * @param {number} mask
     */
    set_contacts(mask) {
        wasm.hexapodgait_set_contacts(this.__wbg_ptr, mask);
    }
    /**
     * Requested gait: 0 tripod, 1 ripple, 2 wave. It takes effect at the next
     * phase boundary.
     * @param {number} kind
     */
    set_gait(kind) {
        wasm.hexapodgait_set_gait(this.__wbg_ptr, kind);
    }
    /**
     * All joint limits (radians). `tibia_fold_femur_down` is the coupled limit
     * that applies while the femur points below level.
     * @param {number} coxa_min
     * @param {number} coxa_max
     * @param {number} femur_min
     * @param {number} femur_max
     * @param {number} tibia_min
     * @param {number} tibia_max
     * @param {number} tibia_fold_femur_down
     */
    set_joint_limits(coxa_min, coxa_max, femur_min, femur_max, tibia_min, tibia_max, tibia_fold_femur_down) {
        wasm.hexapodgait_set_joint_limits(this.__wbg_ptr, coxa_min, coxa_max, femur_min, femur_max, tibia_min, tibia_max, tibia_fold_femur_down);
    }
    /**
     * Body pose in radians / metres (after the balance controller).
     * @param {number} roll
     * @param {number} pitch
     * @param {number} yaw
     * @param {number} shift_x
     * @param {number} shift_z
     */
    set_pose(roll, pitch, yaw, shift_x, shift_z) {
        wasm.hexapodgait_set_pose(this.__wbg_ptr, roll, pitch, yaw, shift_x, shift_z);
    }
    /**
     * Stance-window rule (load check): stance feet stay between `min` and `max`
     * from the coxa axis; the gait steps faster (up to `sprint_hz`), then slows down.
     * @param {number} min
     * @param {number} max
     * @param {number} sprint_hz
     */
    set_reach_window(min, max, sprint_hz) {
        wasm.hexapodgait_set_reach_window(this.__wbg_ptr, min, max, sprint_hz);
    }
    /**
     * @param {number} step_height
     * @param {number} step_hz
     */
    set_step(step_height, step_hz) {
        wasm.hexapodgait_set_step(this.__wbg_ptr, step_height, step_hz);
    }
    /**
     * Terrain mode: touchdown search and per-leg ground offsets.
     * @param {boolean} on
     */
    set_terrain(on) {
        wasm.hexapodgait_set_terrain(this.__wbg_ptr, on);
    }
    /**
     * Joint limit from the CAD: the tibia may not fold further than this (radians).
     * @param {number} min_tibia
     */
    set_tibia_limit(min_tibia) {
        wasm.hexapodgait_set_tibia_limit(this.__wbg_ptr, min_tibia);
    }
    /**
     * < 1 when the stance-window rule is slowing the body down.
     * @returns {number}
     */
    speed_scale() {
        const ret = wasm.hexapodgait_speed_scale(this.__wbg_ptr);
        return ret;
    }
    /**
     * Static stability margin (m) of the stance feet in the level frame, for a
     * centre of mass projected at (com_x, com_z).
     * @param {number} com_x
     * @param {number} com_z
     * @returns {number}
     */
    stability_margin(com_x, com_z) {
        const ret = wasm.hexapodgait_stability_margin(this.__wbg_ptr, com_x, com_z);
        return ret;
    }
    /**
     * Bit i = leg i in stance.
     * @returns {number}
     */
    stance_mask() {
        const ret = wasm.hexapodgait_stance_mask(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Advance the gait. `vx`, `vz` = body velocity (m/s, +Z forward, +X left),
     * `yaw_rate` rad/s (+ = turn left).
     * @param {number} dt
     * @param {number} vx
     * @param {number} vz
     * @param {number} yaw_rate
     */
    update(dt, vx, vz, yaw_rate) {
        wasm.hexapodgait_update(this.__wbg_ptr, dt, vx, vz, yaw_rate);
    }
    /**
     * The body velocity the stance feet are actually pushing (vx, vz, yaw rate),
     * after the command filter and the stance-window rule. Move the body by this
     * and the planted feet don't slide.
     * @returns {Float32Array}
     */
    velocity() {
        const ret = wasm.hexapodgait_velocity(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
}
if (Symbol.dispose) HexapodGait.prototype[Symbol.dispose] = HexapodGait.prototype.free;

export class Terrain {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        TerrainFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_terrain_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    cell() {
        const ret = wasm.terrain_cell(this.__wbg_ptr);
        return ret;
    }
    /**
     * Where the body sits. `feet_body` = [x, y, z] * 6 foot positions in the
     * posed body frame, `feet_xz` = [x, z] * 6 world positions of the feet (from
     * the last frame, for sampling the ground), `support` = bit mask of legs
     * that may carry weight (stance), `yaw` = the pose yaw (rad).
     * Returns [height of the body origin, roll, pitch, contact mask,
     * lowest-foot clearance].
     * @param {Float32Array | number[]} feet_body
     * @param {Float32Array | number[]} feet_xz
     * @param {number} support
     * @param {number} yaw
     * @param {number} roll0
     * @param {number} pitch0
     * @returns {Float32Array}
     */
    fit_body(feet_body, feet_xz, support, yaw, roll0, pitch0) {
        const val3 = +support;
        const val4 = +yaw;
        const val5 = +roll0;
        const val6 = +pitch0;
        let ptr0, len0, ptr1, len1;
        try {
            ptr0 = passArrayF32ToWasm0(feet_body, wasm.__wbindgen_malloc);
            len0 = WASM_VECTOR_LEN;
            ptr1 = passArrayF32ToWasm0(feet_xz, wasm.__wbindgen_malloc);
            len1 = WASM_VECTOR_LEN;
        } catch (e) {
            if (ptr1) { wasm.__wbindgen_free(ptr1, len1 * 4, 4); }
            if (ptr0) { wasm.__wbindgen_free(ptr0, len0 * 4, 4); }
            throw e;
        }
        const ret = wasm.terrain_fit_body(this.__wbg_ptr, ptr0, len0, ptr1, len1, val3, val4, val5, val6);
        var v7 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v7;
    }
    flatten() {
        wasm.terrain_flatten(this.__wbg_ptr);
    }
    /**
     * Rough ground: rolling bumps, a stepped pad, a ramp and rocks. Keeps a
     * flat apron of `keep_flat` metres round the two spawn points (0, +-spawn_z)
     * and along the walls.
     * @param {number} seed
     * @param {number} amplitude
     * @param {number} spawn_z
     * @param {number} keep_flat
     */
    generate_rough(seed, amplitude, spawn_z, keep_flat) {
        wasm.terrain_generate_rough(this.__wbg_ptr, seed, amplitude, spawn_z, keep_flat);
    }
    /**
     * @param {number} x
     * @param {number} z
     * @returns {number}
     */
    height(x, z) {
        const ret = wasm.terrain_height(this.__wbg_ptr, x, z);
        return ret;
    }
    /**
     * All heights, row-major (z rows, x columns), for building the ground mesh.
     * @returns {Float32Array}
     */
    heights() {
        const ret = wasm.terrain_heights(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Heights under several points: [x, z] * n -> [y] * n.
     * @param {Float32Array | number[]} xz
     * @returns {Float32Array}
     */
    heights_at(xz) {
        const ptr0 = passArrayF32ToWasm0(xz, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.terrain_heights_at(this.__wbg_ptr, ptr0, len0);
        var v2 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v2;
    }
    /**
     * Square arena of +-`half` metres with samples every `cell` metres, flat.
     * @param {number} half
     * @param {number} cell
     */
    constructor(half, cell) {
        const ret = wasm.terrain_new(half, cell);
        this.__wbg_ptr = ret;
        TerrainFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {number}
     */
    samples() {
        const ret = wasm.terrain_samples(this.__wbg_ptr);
        return ret >>> 0;
    }
}
if (Symbol.dispose) Terrain.prototype[Symbol.dispose] = Terrain.prototype.free;

export class World {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        WorldFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_world_free(ptr, 0);
    }
    /**
     * Register a robot hit sphere. Returns its id.
     * @param {number} radius
     * @returns {number}
     */
    add_body(radius) {
        const ret = wasm.world_add_body(this.__wbg_ptr, radius);
        return ret >>> 0;
    }
    /**
     * @param {number} minx
     * @param {number} miny
     * @param {number} minz
     * @param {number} maxx
     * @param {number} maxy
     * @param {number} maxz
     */
    add_obstacle(minx, miny, minz, maxx, maxy, maxz) {
        wasm.world_add_obstacle(this.__wbg_ptr, minx, miny, minz, maxx, maxy, maxz);
    }
    /**
     * Ballistic aiming helper: launch pitch (radians, low arc) needed to hit
     * a target `dist` metres away and `dh` metres higher at launch speed `v`,
     * ignoring drag. Returns NaN if out of range.
     * @param {number} dist
     * @param {number} dh
     * @param {number} v
     * @returns {number}
     */
    static ballistic_pitch(dist, dh, v) {
        const ret = wasm.world_ballistic_pitch(dist, dh, v);
        return ret;
    }
    clear_obstacles() {
        wasm.world_clear_obstacles(this.__wbg_ptr);
    }
    clear_projectiles() {
        wasm.world_clear_projectiles(this.__wbg_ptr);
    }
    /**
     * Square arena from -half_size to +half_size on X and Z.
     * @param {number} half_size
     */
    constructor(half_size) {
        const ret = wasm.world_new(half_size);
        this.__wbg_ptr = ret;
        WorldFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {number}
     */
    projectile_count() {
        const ret = wasm.world_projectile_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * [x, y, z, kind] * n for rendering.
     * @returns {Float32Array}
     */
    projectile_state() {
        const ret = wasm.world_projectile_state(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Cast a ray. Returns [distance, body_id or -1 (obstacle/wall), hit_x, hit_y, hit_z].
     * `ignore` is a body id to skip (the robot firing).
     * @param {number} ox
     * @param {number} oy
     * @param {number} oz
     * @param {number} dx
     * @param {number} dy
     * @param {number} dz
     * @param {number} max_dist
     * @param {number} ignore
     * @returns {Float32Array}
     */
    raycast(ox, oy, oz, dx, dy, dz, max_dist, ignore) {
        const ret = wasm.world_raycast(this.__wbg_ptr, ox, oy, oz, dx, dy, dz, max_dist, ignore);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Push a robot footprint (circle on the XZ plane) out of obstacles,
     * other robots and the arena walls. Returns the corrected [x, z].
     * @param {number} id
     * @param {number} x
     * @param {number} z
     * @param {number} radius
     * @returns {Float32Array}
     */
    resolve_body(id, x, z, radius) {
        const ret = wasm.world_resolve_body(this.__wbg_ptr, id, x, z, radius);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Horizontal fan of rays, like an ultrasonic sweep or a 2D LiDAR.
     * Returns [distance, body_id] * rays, from -fov/2 to +fov/2 around `yaw`.
     * @param {number} ox
     * @param {number} oy
     * @param {number} oz
     * @param {number} yaw
     * @param {number} fov
     * @param {number} rays
     * @param {number} max_dist
     * @param {number} ignore
     * @returns {Float32Array}
     */
    scan(ox, oy, oz, yaw, fov, rays, max_dist, ignore) {
        const ret = wasm.world_scan(this.__wbg_ptr, ox, oy, oz, yaw, fov, rays, max_dist, ignore);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * @param {number} id
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {boolean} active
     */
    set_body(id, x, y, z, active) {
        wasm.world_set_body(this.__wbg_ptr, id, x, y, z, active);
    }
    /**
     * Use this ground for projectiles hitting the floor (flat = 0 otherwise).
     * @param {Terrain} t
     */
    set_terrain(t) {
        _assertClass(t, Terrain);
        wasm.world_set_terrain(this.__wbg_ptr, t.__wbg_ptr);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} vx
     * @param {number} vy
     * @param {number} vz
     * @param {number} radius
     * @param {number} drag
     * @param {number} owner
     * @param {number} damage
     * @param {number} kind
     */
    spawn_projectile(x, y, z, vx, vy, vz, radius, drag, owner, damage, kind) {
        wasm.world_spawn_projectile(this.__wbg_ptr, x, y, z, vx, vy, vz, radius, drag, owner, damage, kind);
    }
    /**
     * Advance all projectiles. Uses sub-steps so fast projectiles can't tunnel
     * through thin robots. Returns events as
     * [event_kind, target_body (or -1), damage, x, y, z, projectile_kind, owner] * n.
     * @param {number} dt
     * @returns {Float32Array}
     */
    step(dt) {
        const ret = wasm.world_step(this.__wbg_ptr, dt);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
}
if (Symbol.dispose) World.prototype[Symbol.dispose] = World.prototype.free;

/**
 * Planar 2-link IK used by the robotic arm (shoulder + elbow), returning
 * [shoulder, elbow, clamped] with the elbow bent upward. Shoulder is the
 * elevation above horizontal, elbow is relative to the upper arm.
 * @param {number} reach
 * @param {number} height
 * @param {number} upper
 * @param {number} fore
 * @returns {Float32Array}
 */
export function solve_arm_2link(reach, height, upper, fore) {
    const ret = wasm.solve_arm_2link(reach, height, upper, fore);
    var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
    wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
    return v1;
}

/**
 * Solve many legs at once. `targets` is [x, y, z] * n, the result is
 * [coxa, femur, tibia] * n.
 * @param {Float32Array | number[]} targets
 * @param {number} coxa_len
 * @param {number} femur_len
 * @param {number} tibia_len
 * @returns {Float32Array}
 */
export function solve_legs(targets, coxa_len, femur_len, tibia_len) {
    const val2 = +coxa_len;
    const val3 = +femur_len;
    const val4 = +tibia_len;
    const ptr0 = passArrayF32ToWasm0(targets, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.solve_legs(ptr0, len0, val2, val3, val4);
    var v5 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
    wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
    return v5;
}
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg___wbindgen_throw_4c88d8bf264fd984: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            table.set(0, undefined);
            table.set(1024, undefined);
            table.set(1025, null);
            table.set(1026, true);
            table.set(1027, false);
        },
    };
    return {
        __proto__: null,
        "./robot_core_bg.js": import0,
    };
}

const BalanceFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_balance_free(ptr, 1));
const HexapodGaitFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_hexapodgait_free(ptr, 1));
const TerrainFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_terrain_free(ptr, 1));
const WorldFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_world_free(ptr, 1));

function _assertClass(instance, klass) {
    if (!(instance instanceof klass)) {
        throw new Error(`expected instance of ${klass.name}`);
    }
}

function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

let cachedFloat32ArrayMemory0 = null;
function getFloat32ArrayMemory0() {
    if (cachedFloat32ArrayMemory0 === null || cachedFloat32ArrayMemory0.byteLength === 0) {
        cachedFloat32ArrayMemory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return decodeText(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passArrayF32ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 4, 4) >>> 0;
    getFloat32ArrayMemory0().set(arg, ptr / 4);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(bytes) {
    numBytesDecoded += bytes.length;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true });
        cachedTextDecoder.decode();
        numBytesDecoded = bytes.length;
    }
    return cachedTextDecoder.decode(bytes);
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedFloat32ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (!module.ok) {
            throw new Error(`failed to fetch Wasm: ${module.status} ${module.statusText} fetching '${module.url}'`);
        }

        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

let __wbg_init_promise;
async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;
    if (__wbg_init_promise === undefined) {
        __wbg_init_promise = __wbg_init_once(module_or_path);
        __wbg_init_promise.catch(() => { __wbg_init_promise = undefined; });
    }
    return __wbg_init_promise;
}

async function __wbg_init_once(module_or_path) {

    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('robot_core_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };

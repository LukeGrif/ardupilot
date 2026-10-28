/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Submarine simulator class
*/

#include "SIM_Submarine.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

static Thruster vectored_thrusters[] =
{      //       Motor #     Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor
       Thruster(0,          0,              0,              1.0f,           0,                  -1.0f,              1.0f),
       Thruster(1,          0,              0,              -1.0f,          0,                  -1.0f,              -1.0f),
       Thruster(2,          0,              0,              -1.0f,          0,                  1.0f,               1.0f),
       Thruster(3,          0,              0,              1.0f,           0,                  1.0f,               -1.0f),
       Thruster(4,          1.0f,           0,              0,              -1.0f,              0,                  0),
       Thruster(5,          -1.0f,          0,              0,              -1.0f,              0,                  0)
};


static Thruster vectored_6dof_thrusters[] =
{
       //       Motor #     Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor
       Thruster(0,          0,              0,              1.0f,           0,                  -1.0f,              1.0f),
       Thruster(1,          0,              0,              -1.0f,          0,                  -1.0f,              -1.0f),
       Thruster(2,          0,              0,              -1.0f,          0,                  1.0f,               1.0f),
       Thruster(3,          0,              0,              1.0f,           0,                  1.0f,               -1.0f),
       Thruster(4,          1.0f,           -1.0f,          0,              -1.0f,              0,                  0),
       Thruster(5,          -1.0f,          -1.0f,          0,              -1.0f,              0,                  0),
       Thruster(6,          1.0f,           1.0f,           0,              -1.0f,              0,                  0),
       Thruster(7,          -1.0f,          1.0f,           0,              -1.0f,              0,                  0)
};

Submarine::Submarine(const char *frame_str) :
    Aircraft(frame_str),
    frame(NULL)
{
    frame_height = 0.0;
    ground_behavior = GROUND_BEHAVIOR_NONE;

    // default to vectored frame
    thrusters = vectored_thrusters;
    n_thrusters = 6;

    if (strstr(frame_str, "vectored_6dof")) {
        thrusters = vectored_6dof_thrusters;
        n_thrusters = 8;
    }

    // "_t200": measured Blue Robotics T200 thrust instead of the quadratic
    // approximation, e.g. --model vectored_6dof_t200
    t200_thrust = strstr(frame_str, "_t200") != nullptr;

    lock_step_scheduled = true;
}

/*
  Blue Robotics T200 thrust (kgf) at 16 V for ESC pulses 1100..1900 us in
  4 us steps, from the manufacturer's public performance data (September
  2019). Dead band about 1472-1528 us; thrust grows roughly with pulse^1.7;
  reverse is weaker than forward.
 */
static const float t200_16v_kgf[] = {
    -4.069f, -4.051f, -4.023f, -3.964f, -3.901f, -3.869f, -3.824f, -3.801f, -3.751f, -3.715f,
    -3.656f, -3.588f, -3.520f, -3.452f, -3.397f, -3.311f, -3.252f, -3.202f, -3.107f, -3.039f,
    -2.994f, -2.939f, -2.862f, -2.821f, -2.771f, -2.708f, -2.658f, -2.581f, -2.549f, -2.508f,
    -2.454f, -2.381f, -2.354f, -2.277f, -2.236f, -2.195f, -2.118f, -2.082f, -2.023f, -1.978f,
    -1.941f, -1.860f, -1.814f, -1.764f, -1.701f, -1.669f, -1.610f, -1.556f, -1.515f, -1.488f,
    -1.438f, -1.397f, -1.347f, -1.302f, -1.256f, -1.202f, -1.161f, -1.120f, -1.102f, -1.052f,
    -1.021f, -0.980f, -0.943f, -0.903f, -0.871f, -0.821f, -0.785f, -0.744f, -0.717f, -0.680f,
    -0.649f, -0.617f, -0.581f, -0.540f, -0.513f, -0.481f, -0.445f, -0.417f, -0.386f, -0.354f,
    -0.322f, -0.295f, -0.259f, -0.236f, -0.209f, -0.181f, -0.150f, -0.127f, -0.104f, -0.086f,
    -0.068f, -0.050f, -0.036f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
    0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.041f, 0.054f,
    0.077f, 0.100f, 0.127f, 0.154f, 0.181f, 0.218f, 0.249f, 0.290f, 0.322f, 0.358f,
    0.399f, 0.435f, 0.472f, 0.513f, 0.558f, 0.603f, 0.640f, 0.680f, 0.721f, 0.776f,
    0.816f, 0.871f, 0.907f, 0.948f, 0.993f, 1.034f, 1.098f, 1.143f, 1.184f, 1.238f,
    1.279f, 1.329f, 1.388f, 1.438f, 1.483f, 1.538f, 1.588f, 1.651f, 1.692f, 1.760f,
    1.823f, 1.878f, 1.932f, 1.991f, 2.046f, 2.123f, 2.177f, 2.218f, 2.277f, 2.381f,
    2.431f, 2.517f, 2.576f, 2.654f, 2.731f, 2.758f, 2.839f, 2.889f, 2.976f, 3.053f,
    3.107f, 3.171f, 3.216f, 3.302f, 3.375f, 3.420f, 3.511f, 3.606f, 3.683f, 3.738f,
    3.819f, 3.887f, 3.955f, 4.060f, 4.146f, 4.255f, 4.305f, 4.382f, 4.509f, 4.527f,
    4.649f, 4.708f, 4.785f, 4.835f, 4.931f, 5.012f, 5.076f, 5.144f, 5.175f, 5.221f,
    5.245f,
};

float Submarine::t200_thrust_N(int16_t pwm)
{
    const uint8_t n = ARRAY_SIZE(t200_16v_kgf);
    const float x = (constrain_int16(pwm, 1100, 1900) - 1100) / 4.0f;
    const uint8_t i = MIN(uint8_t(x), uint8_t(n - 2));
    const float frac = x - i;
    return (t200_16v_kgf[i] + frac * (t200_16v_kgf[i+1] - t200_16v_kgf[i])) * GRAVITY_MSS;
}

float Submarine::perpendicular_distance_to_rangefinder_surface() const
{
    const float floor_depth = calculate_sea_floor_depth(position);
    return floor_depth - position.z;
}

// calculate rotational and linear accelerations
void Submarine::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    rot_accel = Vector3f(0,0,0);

    // slight positive buoyancy
    body_accel = dcm.transposed() *  Vector3f(0, 0, -calculate_buoyancy_acceleration());

    for (int i = 0; i < n_thrusters; i++) {
        Thruster t = thrusters[i];
        int16_t pwm = input.servos[t.servo];
        float thrust = 0;
        if (t200_thrust) {
            if (pwm > 1000 && pwm < 2000) {
                thrust = t200_thrust_N(pwm);
            }
        } else {
            float output = 0;
            // if valid pwm and not in the esc deadzone
            // TODO: extract deadzone from parameters/vehicle code
            if (pwm < 2000 && pwm > 1000 && (pwm < 1475 || pwm > 1525)) {
                output = (pwm - 1500) / 400.0; // range -1~1
            }
            thrust = output * fabs(output) * frame_property.thrust; // approximate pwm to thrust function using a quadratic curve
        }
        body_accel += t.linear * thrust / frame_property.weight;
        rot_accel += t.rotational * thrust * frame_property.thruster_mount_radius / frame_property.moment_of_inertia;
    }

    // Limit movement at the sea floor
    const float floor_depth = calculate_sea_floor_depth(position);
    if (position.z > floor_depth && body_accel.z > -GRAVITY_MSS) {
    	body_accel.z = -GRAVITY_MSS;
    }

    // Calculate linear drag forces
    Vector3f linear_drag_forces;
    calculate_drag_force(velocity_air_bf, frame_property.linear_drag_coefficient, linear_drag_forces);
    // Add forces in body frame accel
    body_accel -= linear_drag_forces / frame_property.weight;

    // Calculate angular drag forces
    // TODO: This results in the wrong units. Fix the math.
    Vector3f angular_drag_torque;
    calculate_angular_drag_torque(gyro, frame_property.angular_drag_coefficient, angular_drag_torque);

    // Calculate torque induced by buoyancy foams on the frame
    Vector3f buoyancy_torque;
    calculate_buoyancy_torque(buoyancy_torque);
    // Add forces in body frame accel
    rot_accel -= angular_drag_torque / frame_property.moment_of_inertia;
    rot_accel += buoyancy_torque / frame_property.moment_of_inertia;
    add_shove_forces(rot_accel, body_accel);
}


/**
 * @brief Calculate the torque induced by buoyancy foam
 *
 * @param torque Output torques
 */
void Submarine::calculate_buoyancy_torque(Vector3f &torque)
{
    // Let's assume 2 Liters water displacement at the top, and ~ 2kg of weight at the bottom.
    const Vector3f force_up(0,0,-40); // 40 N upwards
    const Vector3f force_position = dcm.transposed() * Vector3f(0, 0, 0.15); // offset in meters
    torque = force_position % force_up;
}


/**
 * @brief Calculate sea floor depth from submarine position
 *          This creates a non planar floor for rangefinder sensor test
 *          TODO: Create a better sea floor with procedural generatation
 *
 * @param position
 * @return float
 */
float Submarine::calculate_sea_floor_depth(const Vector3d &/*position*/) const
{
    return 50;
}

/**
 * @brief Calculate drag force against body
 *
 * @param velocity Body frame velocity of fluid
 * @param drag_coefficient Drag coefficient of body
 * @param force Output forces
 * $ F_D = rho * v^2 * A * C_D / 2 $
 * rho = water density (kg/m^3), V = velocity (m/s), A = area (m^2), C_D = drag_coefficient
 */
void Submarine::calculate_drag_force(const Vector3f &velocity, const Vector3f &drag_coefficient, Vector3f &force) const
{
    /**
     * @brief It's necessary to keep the velocity orientation from the body frame.
     *     To do so, a mathematical artifice is used to do velocity square but without loosing the direction.
     *  $(|V|/V)*V^2$ = $|V|*V$
     */
    const Vector3f velocity_2(
        fabsf(velocity.x) * velocity.x,
        fabsf(velocity.y) * velocity.y,
        fabsf(velocity.z) * velocity.z
    );

    force = (velocity_2 * water_density) * frame_property.equivalent_sphere_area / 2.0f;
    force *= drag_coefficient;
}

/**
 * @brief Calculate angular drag torque using the equivalente sphere area and assuming a laminar external flow.
 *
 *  $F_D = C_D*A*\rho*V^2/2$
 * where:
 *      $F_D$ is the drag force
 *      $C_D$ is the drag coefficient
 *      $A$ is the surface area in contact with the fluid
 *      $/rho$ is the fluid density (1000kg/m³ for water)
 *      $V$ is the fluid velocity velocity relative to the surface
 *
 * @param angular_velocity Body frame velocity of fluid
 * @param drag_coefficient Rotational drag coefficient of body
 */
void Submarine::calculate_angular_drag_torque(const Vector3f &angular_velocity, const Vector3f &drag_coefficient, Vector3f &torque) const
{
     /**
     * @brief It's necessary to keep the velocity orientation from the body frame.
     *     To do so, a mathematical artifice is used to do velocity square but without loosing the direction.
     *  $(|V|/V)*V^2$ = $|V|*V$
     */
    Vector3f v_2(
        fabsf(angular_velocity.x) * angular_velocity.x,
        fabsf(angular_velocity.y) * angular_velocity.y,
        fabsf(angular_velocity.z) * angular_velocity.z
    );
    Vector3f f_d = v_2 *= drag_coefficient * frame_property.equivalent_sphere_area * 1000 / 2;
    torque = f_d * frame_property.equivalent_sphere_radius;
}


/**
* @brief Calculate buoyancy force of the frame
*
* @return float
*/
float Submarine::calculate_buoyancy_acceleration()
{
    float below_water_level = position.z - frame_property.height/2;

    // Completely above water level
    if (below_water_level < 0) {
        return 0.0f;
    }

    // Completely below water level
    if (below_water_level > frame_property.height/2) {
        return GRAVITY_MSS + sitl->buoyancy / frame_property.mass;
    }

    // bouyant force is proportional to fraction of height in water
    return GRAVITY_MSS + (sitl->buoyancy * below_water_level/frame_property.height) / frame_property.mass;
};

/*
  update the Submarine simulation by one time step
 */
void Submarine::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
   return true if we are on the ground
*/
bool Submarine::on_ground() const
{
	return false;
}

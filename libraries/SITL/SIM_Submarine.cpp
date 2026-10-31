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
  Blue Robotics T200 thrust (kgf) for ESC pulses 1100..1900 us in 4 us steps,
  at 10, 12, ... 20 V battery, from the manufacturer's public performance data
  (September 2019). The dead band is about 1472-1528 us at 16 V and widens as
  the voltage falls; reverse is about 22% weaker than forward.
 */
static const uint8_t t200_volts_first = 10;
static const uint8_t t200_volts_step = 2;
static const float t200_kgf[][201] = {
    {   // 10 V
        -2.309f, -2.304f, -2.295f, -2.282f, -2.223f, -2.182f, -2.164f, -2.132f, -2.109f, -2.055f,
        -2.041f, -1.987f, -1.955f, -1.928f, -1.891f, -1.837f, -1.810f, -1.792f, -1.755f, -1.715f,
        -1.683f, -1.651f, -1.619f, -1.592f, -1.556f, -1.538f, -1.501f, -1.470f, -1.442f, -1.411f,
        -1.361f, -1.329f, -1.320f, -1.288f, -1.256f, -1.220f, -1.188f, -1.166f, -1.139f, -1.098f,
        -1.075f, -1.057f, -1.016f, -0.989f, -0.971f, -0.930f, -0.907f, -0.880f, -0.853f, -0.826f,
        -0.798f, -0.780f, -0.753f, -0.721f, -0.699f, -0.671f, -0.653f, -0.630f, -0.612f, -0.581f,
        -0.553f, -0.531f, -0.513f, -0.485f, -0.467f, -0.445f, -0.422f, -0.399f, -0.381f, -0.354f,
        -0.336f, -0.313f, -0.295f, -0.277f, -0.249f, -0.236f, -0.213f, -0.200f, -0.181f, -0.163f,
        -0.154f, -0.136f, -0.118f, -0.100f, -0.086f, -0.073f, -0.064f, -0.054f, -0.045f, -0.032f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.041f, 0.054f, 0.068f, 0.077f, 0.091f, 0.109f, 0.127f, 0.145f, 0.163f,
        0.181f, 0.204f, 0.222f, 0.245f, 0.263f, 0.290f, 0.313f, 0.336f, 0.363f, 0.386f,
        0.413f, 0.445f, 0.467f, 0.494f, 0.522f, 0.549f, 0.581f, 0.608f, 0.640f, 0.685f,
        0.703f, 0.726f, 0.762f, 0.798f, 0.826f, 0.853f, 0.889f, 0.921f, 0.948f, 0.975f,
        0.971f, 1.048f, 1.075f, 1.111f, 1.166f, 1.184f, 1.207f, 1.238f, 1.275f, 1.311f,
        1.356f, 1.411f, 1.429f, 1.479f, 1.524f, 1.547f, 1.579f, 1.624f, 1.656f, 1.692f,
        1.728f, 1.774f, 1.801f, 1.846f, 1.878f, 1.914f, 1.991f, 1.996f, 2.059f, 2.091f,
        2.123f, 2.168f, 2.213f, 2.272f, 2.304f, 2.363f, 2.409f, 2.454f, 2.481f, 2.554f,
        2.581f, 2.617f, 2.649f, 2.685f, 2.735f, 2.771f, 2.821f, 2.876f, 2.894f, 2.917f,
        2.926f,
    },
    {   // 12 V
        -2.904f, -2.917f, -2.894f, -2.830f, -2.785f, -2.762f, -2.722f, -2.672f, -2.599f, -2.595f,
        -2.563f, -2.495f, -2.440f, -2.431f, -2.386f, -2.341f, -2.304f, -2.254f, -2.227f, -2.182f,
        -2.141f, -2.096f, -2.068f, -2.009f, -1.982f, -1.946f, -1.882f, -1.846f, -1.805f, -1.778f,
        -1.733f, -1.665f, -1.651f, -1.610f, -1.560f, -1.533f, -1.492f, -1.470f, -1.438f, -1.397f,
        -1.370f, -1.334f, -1.293f, -1.279f, -1.220f, -1.193f, -1.152f, -1.116f, -1.084f, -1.043f,
        -1.021f, -0.993f, -0.962f, -0.930f, -0.903f, -0.871f, -0.835f, -0.794f, -0.771f, -0.739f,
        -0.721f, -0.694f, -0.658f, -0.640f, -0.603f, -0.572f, -0.544f, -0.517f, -0.494f, -0.472f,
        -0.440f, -0.417f, -0.390f, -0.367f, -0.340f, -0.318f, -0.295f, -0.268f, -0.245f, -0.227f,
        -0.204f, -0.177f, -0.163f, -0.145f, -0.122f, -0.109f, -0.091f, -0.073f, -0.059f, -0.045f,
        -0.036f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.041f, 0.054f, 0.073f, 0.095f, 0.113f, 0.132f, 0.154f, 0.177f, 0.200f, 0.222f,
        0.254f, 0.277f, 0.308f, 0.331f, 0.367f, 0.395f, 0.426f, 0.458f, 0.490f, 0.517f,
        0.549f, 0.590f, 0.626f, 0.653f, 0.685f, 0.712f, 0.757f, 0.789f, 0.830f, 0.857f,
        0.894f, 0.930f, 0.966f, 1.002f, 1.039f, 1.084f, 1.139f, 1.161f, 1.198f, 1.234f,
        1.279f, 1.311f, 1.352f, 1.402f, 1.433f, 1.483f, 1.529f, 1.565f, 1.633f, 1.674f,
        1.710f, 1.769f, 1.819f, 1.846f, 1.905f, 1.923f, 1.964f, 2.032f, 2.087f, 2.132f,
        2.177f, 2.236f, 2.272f, 2.332f, 2.400f, 2.463f, 2.508f, 2.563f, 2.617f, 2.654f,
        2.712f, 2.776f, 2.844f, 2.871f, 2.930f, 3.007f, 3.044f, 3.084f, 3.157f, 3.225f,
        3.261f, 3.316f, 3.379f, 3.402f, 3.447f, 3.502f, 3.570f, 3.638f, 3.706f, 3.692f,
        3.714f,
    },
    {   // 14 V
        -3.517f, -3.497f, -3.493f, -3.452f, -3.402f, -3.357f, -3.293f, -3.248f, -3.189f, -3.139f,
        -3.098f, -3.057f, -3.003f, -2.939f, -2.885f, -2.849f, -2.780f, -2.762f, -2.690f, -2.644f,
        -2.595f, -2.526f, -2.490f, -2.454f, -2.409f, -2.350f, -2.336f, -2.259f, -2.204f, -2.177f,
        -2.118f, -2.055f, -2.028f, -1.987f, -1.910f, -1.891f, -1.819f, -1.764f, -1.719f, -1.683f,
        -1.633f, -1.579f, -1.565f, -1.520f, -1.479f, -1.442f, -1.402f, -1.370f, -1.325f, -1.284f,
        -1.238f, -1.193f, -1.166f, -1.125f, -1.089f, -1.052f, -1.016f, -0.984f, -0.953f, -0.916f,
        -0.884f, -0.848f, -0.807f, -0.767f, -0.739f, -0.703f, -0.680f, -0.649f, -0.621f, -0.585f,
        -0.549f, -0.517f, -0.490f, -0.463f, -0.426f, -0.399f, -0.372f, -0.349f, -0.318f, -0.286f,
        -0.259f, -0.236f, -0.209f, -0.186f, -0.163f, -0.145f, -0.122f, -0.100f, -0.082f, -0.068f,
        -0.045f, -0.032f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.045f,
        0.064f, 0.082f, 0.100f, 0.122f, 0.145f, 0.177f, 0.200f, 0.227f, 0.263f, 0.290f,
        0.327f, 0.363f, 0.395f, 0.426f, 0.463f, 0.499f, 0.531f, 0.576f, 0.621f, 0.644f,
        0.690f, 0.726f, 0.767f, 0.826f, 0.848f, 0.889f, 0.921f, 0.966f, 0.998f, 1.052f,
        1.093f, 1.143f, 1.202f, 1.234f, 1.279f, 1.325f, 1.365f, 1.406f, 1.461f, 1.510f,
        1.551f, 1.606f, 1.651f, 1.710f, 1.760f, 1.814f, 1.851f, 1.910f, 1.964f, 2.005f,
        2.087f, 2.118f, 2.159f, 2.245f, 2.268f, 2.341f, 2.427f, 2.504f, 2.563f, 2.635f,
        2.658f, 2.758f, 2.780f, 2.880f, 2.935f, 2.994f, 3.048f, 3.130f, 3.189f, 3.234f,
        3.325f, 3.361f, 3.420f, 3.493f, 3.565f, 3.624f, 3.688f, 3.769f, 3.837f, 3.924f,
        3.982f, 4.028f, 4.109f, 4.155f, 4.214f, 4.305f, 4.382f, 4.423f, 4.509f, 4.527f,
        4.524f,
    },
    {   // 16 V
        -4.069f, -4.051f, -4.023f, -3.964f, -3.901f, -3.869f, -3.824f, -3.801f, -3.751f, -3.715f,
        -3.656f, -3.588f, -3.520f, -3.452f, -3.397f, -3.311f, -3.252f, -3.202f, -3.107f, -3.039f,
        -2.994f, -2.939f, -2.862f, -2.821f, -2.771f, -2.708f, -2.658f, -2.581f, -2.549f, -2.508f,
        -2.454f, -2.381f, -2.354f, -2.277f, -2.236f, -2.195f, -2.118f, -2.082f, -2.023f, -1.978f,
        -1.941f, -1.860f, -1.814f, -1.764f, -1.701f, -1.669f, -1.610f, -1.556f, -1.515f, -1.488f,
        -1.438f, -1.397f, -1.347f, -1.302f, -1.256f, -1.202f, -1.161f, -1.120f, -1.102f, -1.052f,
        -1.021f, -0.980f, -0.944f, -0.903f, -0.871f, -0.821f, -0.785f, -0.744f, -0.717f, -0.680f,
        -0.649f, -0.617f, -0.581f, -0.540f, -0.513f, -0.481f, -0.445f, -0.417f, -0.386f, -0.354f,
        -0.322f, -0.295f, -0.259f, -0.236f, -0.209f, -0.181f, -0.150f, -0.127f, -0.104f, -0.086f,
        -0.068f, -0.050f, -0.036f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.041f, 0.054f,
        0.077f, 0.100f, 0.127f, 0.154f, 0.181f, 0.218f, 0.249f, 0.290f, 0.322f, 0.358f,
        0.399f, 0.435f, 0.472f, 0.513f, 0.558f, 0.603f, 0.640f, 0.680f, 0.721f, 0.776f,
        0.817f, 0.871f, 0.907f, 0.948f, 0.993f, 1.034f, 1.098f, 1.143f, 1.184f, 1.238f,
        1.279f, 1.329f, 1.388f, 1.438f, 1.483f, 1.538f, 1.588f, 1.651f, 1.692f, 1.760f,
        1.823f, 1.878f, 1.932f, 1.991f, 2.046f, 2.123f, 2.177f, 2.218f, 2.277f, 2.381f,
        2.431f, 2.517f, 2.576f, 2.654f, 2.731f, 2.758f, 2.840f, 2.889f, 2.976f, 3.053f,
        3.107f, 3.171f, 3.216f, 3.302f, 3.375f, 3.420f, 3.511f, 3.606f, 3.683f, 3.738f,
        3.819f, 3.887f, 3.955f, 4.060f, 4.146f, 4.255f, 4.305f, 4.382f, 4.509f, 4.527f,
        4.649f, 4.708f, 4.785f, 4.835f, 4.931f, 5.012f, 5.076f, 5.144f, 5.176f, 5.221f,
        5.245f,
    },
    {   // 18 V
        -4.590f, -4.545f, -4.531f, -4.477f, -4.418f, -4.350f, -4.345f, -4.168f, -4.182f, -4.105f,
        -4.037f, -3.992f, -3.914f, -3.860f, -3.810f, -3.715f, -3.683f, -3.556f, -3.543f, -3.425f,
        -3.357f, -3.266f, -3.193f, -3.094f, -3.030f, -2.957f, -2.880f, -2.844f, -2.776f, -2.699f,
        -2.658f, -2.586f, -2.526f, -2.477f, -2.422f, -2.363f, -2.318f, -2.254f, -2.209f, -2.159f,
        -2.109f, -2.041f, -1.996f, -1.941f, -1.882f, -1.814f, -1.760f, -1.719f, -1.660f, -1.610f,
        -1.565f, -1.520f, -1.465f, -1.424f, -1.379f, -1.329f, -1.275f, -1.225f, -1.175f, -1.134f,
        -1.084f, -1.048f, -1.012f, -0.971f, -0.934f, -0.894f, -0.857f, -0.812f, -0.771f, -0.739f,
        -0.699f, -0.671f, -0.635f, -0.599f, -0.558f, -0.531f, -0.494f, -0.463f, -0.431f, -0.399f,
        -0.367f, -0.336f, -0.304f, -0.272f, -0.240f, -0.209f, -0.181f, -0.154f, -0.127f, -0.104f,
        -0.077f, -0.059f, -0.045f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.045f, 0.068f,
        0.091f, 0.122f, 0.150f, 0.181f, 0.213f, 0.249f, 0.286f, 0.318f, 0.358f, 0.390f,
        0.426f, 0.472f, 0.508f, 0.549f, 0.585f, 0.626f, 0.667f, 0.699f, 0.757f, 0.803f,
        0.839f, 0.880f, 0.916f, 0.966f, 1.016f, 1.075f, 1.134f, 1.179f, 1.225f, 1.279f,
        1.320f, 1.383f, 1.424f, 1.488f, 1.538f, 1.597f, 1.678f, 1.742f, 1.787f, 1.842f,
        1.919f, 1.973f, 2.037f, 2.082f, 2.173f, 2.218f, 2.282f, 2.363f, 2.431f, 2.504f,
        2.586f, 2.644f, 2.744f, 2.794f, 2.867f, 2.926f, 3.012f, 3.103f, 3.148f, 3.207f,
        3.266f, 3.393f, 3.456f, 3.538f, 3.629f, 3.724f, 3.792f, 3.924f, 4.019f, 4.141f,
        4.209f, 4.318f, 4.409f, 4.536f, 4.668f, 4.695f, 4.862f, 4.917f, 5.017f, 5.121f,
        5.207f, 5.293f, 5.384f, 5.497f, 5.597f, 5.665f, 5.770f, 5.824f, 5.897f, 5.983f,
        6.016f,
    },
    {   // 20 V
        -5.036f, -4.958f, -4.921f, -4.844f, -4.795f, -4.745f, -4.680f, -4.577f, -4.463f, -4.404f,
        -4.350f, -4.228f, -4.159f, -4.119f, -4.019f, -3.955f, -3.887f, -3.810f, -3.710f, -3.660f,
        -3.556f, -3.479f, -3.357f, -3.288f, -3.162f, -3.125f, -3.003f, -2.944f, -2.871f, -2.776f,
        -2.735f, -2.672f, -2.604f, -2.545f, -2.499f, -2.436f, -2.404f, -2.313f, -2.263f, -2.200f,
        -2.141f, -2.109f, -2.041f, -1.969f, -1.914f, -1.855f, -1.796f, -1.728f, -1.674f, -1.637f,
        -1.583f, -1.520f, -1.474f, -1.411f, -1.374f, -1.325f, -1.270f, -1.220f, -1.170f, -1.107f,
        -1.066f, -1.021f, -0.989f, -0.944f, -0.907f, -0.875f, -0.830f, -0.780f, -0.753f, -0.717f,
        -0.690f, -0.658f, -0.630f, -0.590f, -0.558f, -0.531f, -0.503f, -0.472f, -0.445f, -0.413f,
        -0.386f, -0.358f, -0.322f, -0.295f, -0.263f, -0.236f, -0.204f, -0.172f, -0.145f, -0.122f,
        -0.091f, -0.073f, -0.054f, -0.036f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
        0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.059f, 0.086f,
        0.113f, 0.150f, 0.181f, 0.213f, 0.249f, 0.286f, 0.322f, 0.363f, 0.390f, 0.426f,
        0.467f, 0.499f, 0.535f, 0.572f, 0.608f, 0.640f, 0.662f, 0.699f, 0.739f, 0.780f,
        0.817f, 0.862f, 0.889f, 0.934f, 0.975f, 1.021f, 1.061f, 1.120f, 1.170f, 1.220f,
        1.279f, 1.311f, 1.379f, 1.438f, 1.529f, 1.583f, 1.642f, 1.737f, 1.801f, 1.846f,
        1.919f, 1.982f, 2.068f, 2.118f, 2.200f, 2.250f, 2.327f, 2.436f, 2.522f, 2.608f,
        2.663f, 2.762f, 2.826f, 2.908f, 2.994f, 3.057f, 3.134f, 3.220f, 3.280f, 3.357f,
        3.443f, 3.547f, 3.642f, 3.724f, 3.846f, 3.964f, 4.101f, 4.209f, 4.327f, 4.423f,
        4.554f, 4.699f, 4.822f, 4.931f, 5.021f, 5.144f, 5.203f, 5.393f, 5.520f, 5.620f,
        5.711f, 5.815f, 5.928f, 6.060f, 6.128f, 6.255f, 6.364f, 6.459f, 6.554f, 6.622f,
        6.722f,
    },
};

// thrust at the simulated battery voltage (SIM_BATT_VOLTAGE), interpolated
// between the measured voltages
float Submarine::t200_thrust_N(int16_t pwm) const
{
    const uint8_t n = ARRAY_SIZE(t200_kgf[0]);
    const uint8_t nv = ARRAY_SIZE(t200_kgf);
    const float x = (constrain_int16(pwm, 1100, 1900) - 1100) / 4.0f;
    const uint8_t i = MIN(uint8_t(x), uint8_t(n - 2));
    const float frac = x - i;
    const float vx = constrain_float((sitl->batt_voltage - t200_volts_first) / t200_volts_step, 0.0f, nv - 1.0f);
    const uint8_t j = MIN(uint8_t(vx), uint8_t(nv - 2));
    const float vfrac = vx - j;
    const float lo = t200_kgf[j][i] + frac * (t200_kgf[j][i+1] - t200_kgf[j][i]);
    const float hi = t200_kgf[j+1][i] + frac * (t200_kgf[j+1][i+1] - t200_kgf[j+1][i]);
    return (lo + vfrac * (hi - lo)) * GRAVITY_MSS;
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

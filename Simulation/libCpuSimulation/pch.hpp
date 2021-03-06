#pragma once

#define SIMULATION_IMPL
#define _CRT_SECURE_NO_WARNINGS
#define GLM_FORCE_SWIZZLE

#include <cfloat>
#include <cassert>
#include <cstdint>

#include <vector>
#include <memory>
#include <atomic>

#include <algorithm>
#include <numeric>
#include <execution>

#include <glm/glm.hpp>
#include <glm/matrix.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>

#include "../Simulation.hpp"
#pragma once
#define GLEW_STATIC
#define FREEGLUT_LIB_PRAGMAS 0
#define FREEGLUT_STATIC
#define _CRT_SECURE_NO_WARNINGS

#define RENDERING_IMPL

#include <cstdio>

#include <stdexcept>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <execution>

#include <GL/glew.h>
#include <GL/wglew.h>
#include <GL/freeglut.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../../Simulation/Simulation.hpp"
#include "../Rendering.hpp"
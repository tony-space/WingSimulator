#pragma once
#include "pch.hpp"

namespace wing2d
{
	namespace rendering
	{
		namespace opengl
		{
			class CShaderProgram
			{
			public:
				class ProgramSwitcher
				{
				public:
					ProgramSwitcher(GLuint program);
					~ProgramSwitcher();
				private:
					GLuint m_previous = -1;
				};

				CShaderProgram(const std::string& vertexPath, const std::string& fragmentPath);
				~CShaderProgram();
				void SetUniform(const std::string& name, float value);
				void SetUniform(const std::string& name, const glm::vec3& value);
				GLint GetAttributeLocation(const std::string& name);

				ProgramSwitcher Activate() const { return ProgramSwitcher(m_program); }
			private:
				GLuint m_program;
				std::map<std::string, GLint> m_uniformLocationCache;
				std::map<std::string, GLint> m_attribLocationCache;
			};
		}
	}
}
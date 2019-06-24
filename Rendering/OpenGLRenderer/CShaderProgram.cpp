#include "pch.hpp"
#include "CShaderProgram.hpp"

using namespace wing2d::rendering::opengl;

class CSmartFile
{
public:
	CSmartFile(const char* path)
	{
		m_pFile = fopen(path, "rb");
		if (m_pFile == nullptr)
			throw std::invalid_argument("could not open the file");
	}

	~CSmartFile()
	{
		fclose(m_pFile);
	}

	operator FILE* () const
	{
		return m_pFile;
	}
private:
	FILE* m_pFile;
};

class CSmartShader
{
public:
	CSmartShader(GLenum type)
	{
		m_shader = glCreateShader(type);
		if (m_shader == 0) throw std::runtime_error("glCreateShader failed");

	}

	void CompileSource(const std::string& src)
	{
		auto srcPtr = src.c_str();
		glShaderSource(m_shader, 1, &srcPtr, nullptr);
		auto status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glShaderSource failed");

		glCompileShader(m_shader);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glCompileShader failed");

		GLint compilationStatus;
		glGetShaderiv(m_shader, GL_COMPILE_STATUS, &compilationStatus);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetShaderiv failed");

		char buffer[1024];
		GLsizei logSize;
		glGetShaderInfoLog(m_shader, sizeof(buffer), &logSize, buffer);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetShaderInfoLog failed");
		printf("Shader compilation log:\r\n%s\r\n", buffer);

		if (!compilationStatus) throw std::runtime_error("Compilation failed. Check debug messages");
	}

	~CSmartShader()
	{
		glDeleteShader(m_shader);
		auto status = glGetError();
		if (status != GL_NO_ERROR) fprintf(stderr, "glDeleteShader failed\r\n");
	}

	operator GLuint() const
	{
		return m_shader;
	}
private:
	GLuint m_shader;
};

static std::string ReadTextFile(const std::string& path)
{
	CSmartFile file(path.c_str());

	auto status = fseek(file, 0, SEEK_END);
	if (status) throw std::runtime_error("fseek failed");
	size_t size = ftell(file);
	status = fseek(file, 0, SEEK_SET);
	if (status) throw std::runtime_error("fseek failed");

	std::vector<char> symbols(size);
	auto bytesRead = fread(symbols.data(), sizeof(char), size, file);
	if (bytesRead != size) throw std::runtime_error("fread failed");

	return std::string(symbols.begin(), symbols.end());
}

CShaderProgram::CShaderProgram(const std::string& vertexPath, const std::string& fragmentPath)
{
	try
	{
		m_program = glCreateProgram();
		if (m_program == 0) throw std::runtime_error("glCreateProgram failed");

		auto status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glCreateProgram failed");

		auto vertexCode = ReadTextFile(vertexPath);
		auto fragmentCode = ReadTextFile(fragmentPath);

		CSmartShader vertexShader(GL_VERTEX_SHADER);
		vertexShader.CompileSource(vertexCode);

		CSmartShader fragmentShader(GL_FRAGMENT_SHADER);
		fragmentShader.CompileSource(fragmentCode);

		glAttachShader(m_program, vertexShader);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glAttachShader failed");

		glAttachShader(m_program, fragmentShader);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glAttachShader failed");

		glLinkProgram(m_program);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glLinkProgram failed");

		GLint linkStatus;
		glGetProgramiv(m_program, GL_LINK_STATUS, &linkStatus);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetProgramiv failed");

		char buffer[1024];
		GLsizei logSize;
		glGetProgramInfoLog(m_program, sizeof(buffer), &logSize, buffer);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetProgramInfoLog failed");
		printf("Program linking log:\r\n%s\r\n", buffer);

		if (!linkStatus) throw std::runtime_error("Linking failed. Check debug messages");

		glValidateProgram(m_program);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glValidateProgram failed");

		GLint validationStatus;
		glGetProgramiv(m_program, GL_VALIDATE_STATUS, &validationStatus);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetProgramiv failed");

		glGetProgramInfoLog(m_program, sizeof(buffer), &logSize, buffer);
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetProgramInfoLog failed");
		printf("Program validation log:\r\n%s\r\n", buffer);

		if (!validationStatus) throw std::runtime_error("Validation failed. Check debug messages");
	}
	catch (...)
	{
		if (m_program)
		{
			glDeleteProgram(m_program);
			m_program = 0;
		}
		throw;
	}
}

CShaderProgram::~CShaderProgram()
{
	glDeleteProgram(m_program);
	auto status = glGetError();
	if (status != GL_NO_ERROR) fprintf(stderr, "glDeleteProgram failed\r\n");
}

void CShaderProgram::SetUniform(const std::string& name, float value)
{
	GLenum status;
	GLint location;
	auto it = m_uniformLocationCache.find(name);
	if (it != m_uniformLocationCache.end())
		location = it->second;
	else
	{
		location = glGetUniformLocation(m_program, name.c_str());
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetUniformLocation failed");

		m_uniformLocationCache[name] = location;
	}

	glUniform1f(location, value);
	status = glGetError();
	if (status != GL_NO_ERROR) throw std::runtime_error("glUniform1f failed");
}

void CShaderProgram::SetUniform(const std::string& name, const glm::vec3& value)
{
	GLenum status;
	GLint location;
	auto it = m_uniformLocationCache.find(name);
	if (it != m_uniformLocationCache.end())
		location = it->second;
	else
	{
		location = glGetUniformLocation(m_program, name.c_str());
		status = glGetError();
		if (status != GL_NO_ERROR) throw std::runtime_error("glGetUniformLocation failed");

		m_uniformLocationCache[name] = location;
	}

	glUniform3f(location, value.x, value.y, value.z);
	status = glGetError();
	if (status != GL_NO_ERROR) throw std::runtime_error("glUniform3f failed");
}

GLint CShaderProgram::GetAttributeLocation(const std::string & name)
{
	auto it = m_attribLocationCache.find(name);
	if (it != m_attribLocationCache.end())
		return it->second;
	else
	{
		auto location = glGetAttribLocation(m_program, name.c_str());
		auto status = glGetError();
		if(status != GL_NO_ERROR) throw std::runtime_error("GetAttributeLocation failed");

		m_attribLocationCache[name] = location;
		return location;
	}
}

CShaderProgram::ProgramSwitcher::ProgramSwitcher(GLuint program)
{
	GLint temp;
	glGetIntegerv(GL_CURRENT_PROGRAM, &temp);
	auto status = glGetError();
	if (status != GL_NO_ERROR) throw std::runtime_error("glGetIntegerv failed");
	m_previous = GLuint(temp);

	glUseProgram(program);
	status = glGetError();
	if (status != GL_NO_ERROR) throw std::runtime_error("glUseProgram failed");
}

CShaderProgram::ProgramSwitcher::~ProgramSwitcher()
{
	glUseProgram(m_previous);
	auto status = glGetError();
	if (status != GL_NO_ERROR) fprintf(stderr, "glUseProgram failed\r\n");
}

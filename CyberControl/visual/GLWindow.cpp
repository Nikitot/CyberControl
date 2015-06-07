#define WIN32_LEAN_AND_MEAN 1
#include "GLWindow.h"
#include "../cc/stdafx.h"


static const uint8_t INPUT_UP = 0, INPUT_DOWN = 1, INPUT_PRESSED = 2;

static const char GLWINDOW_CLASS_NAME[] = "GLWindow_class";

static HINSTANCE g_hInstance = NULL;
static HWND      g_hWnd      = NULL;
static HDC       g_hDC       = NULL;
static HGLRC     g_hRC       = NULL;

// обработчик сообщений окна
static LRESULT CALLBACK GLWindowProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// хранение стейта окна и ввода
static GLWindow g_window;
static Input    g_input;

static double        g_timerFrequency = 0.0;
static LARGE_INTEGER g_qpc;

// количество вершин в нашей геометрии, у нас простой треугольник
static int MESH_VERTEX_COUNT = 0;

// размер одной вершины меша в байтах - 6 float на позицию и на цвет вершины
static const int VERTEX_SIZE = 6 * sizeof(float);

// смещения данных внутри вершины
static const int VERTEX_POSITION_OFFSET = 0;
static const int VERTEX_COLOR_OFFSET = 3 * sizeof(float);

// пременные для хранения идентификаторов шейдерной программы и шейдеров
GLuint shaderProgram = 0, vertexShader = 0, fragmentShader = 0;

// переменные для хранения идентификаторов VAO и VBO
static GLuint meshVAO, meshVBO;

// подготовим данные для вывода треугольника, всего 3 вершины
float *pointsMesh;

void setPointsMesh(std::vector<float> points){
	pointsMesh = (float *)malloc(points.size()*sizeof(float));
	for (int i = 0; i < points.size(); i++){
		pointsMesh[i] = points.at(i);
		std::cout << pointsMesh[i] << std::endl;
	}

	MESH_VERTEX_COUNT = points.size();

}

// построение перспективной матрицы
static void Matrix4Perspective(float *M, float fovy, float aspect, float znear, float zfar)
{
	float f = 1 / tanf(fovy / 2),
		A = (zfar + znear) / (znear - zfar),
		B = (2 * zfar * znear) / (znear - zfar);

	M[0] = f / aspect; M[1] = 0; M[2] = 0; M[3] = 0;
	M[4] = 0;          M[5] = f; M[6] = 0; M[7] = 0;
	M[8] = 0;          M[9] = 0; M[10] = A; M[11] = B;
	M[12] = 0;          M[13] = 0; M[14] = -1; M[15] = 0;
}


// инициализаця OpenGL
bool GLWindowInit(const GLWindow *window)
{
	ASSERT(window);

	uint8_t  *shaderSource;
	uint32_t sourceLength;

	float projectionMatrix[16];
	GLint projectionMatrixLocation, positionLocation, colorLocation;

	// устанавливаем вьюпорт на все окно
	glViewport(0, 0, window->width, window->height);

	// параметры OpenGL
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClearDepth(1.0f);



	// создадим шейдерную программу и шейдеры для нее
	shaderProgram = glCreateProgram();
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	// загрузим вершинный шейдер
	if (!LoadFile("data/lesson.vs", true, &shaderSource, &sourceLength))
		return false;

	// зададим шейдеру исходный код и скомпилируем его
	glShaderSource(vertexShader, 1, (const GLchar**)&shaderSource, (const GLint*)&sourceLength);
	glCompileShader(vertexShader);

	delete[] shaderSource;

	// проверим статус шейдера
	if (ShaderStatus(vertexShader, GL_COMPILE_STATUS) != GL_TRUE)
		return false;

	// загрузим фрагментный шейдер
	if (!LoadFile("data/lesson.fs", true, &shaderSource, &sourceLength))
		return false;

	// зададим шейдеру исходный код и скомпилируем его
	glShaderSource(fragmentShader, 1, (const GLchar**)&shaderSource, (const GLint*)&sourceLength);
	glCompileShader(fragmentShader);

	delete[] shaderSource;

	// проверим статус шейдера
	if (ShaderStatus(fragmentShader, GL_COMPILE_STATUS) != GL_TRUE)
		return false;

	// присоеденим загруженные шейдеры к шейдерной программе
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);

	// проверим не было ли ошибок
	OPENGL_CHECK_FOR_ERRORS();

	// линковка шейдерной программы и проверка статуса линковки
	glLinkProgram(shaderProgram);
	if (ShaderProgramStatus(shaderProgram, GL_LINK_STATUS) != GL_TRUE)
		return false;

	// сделаем шейдер активным
	glUseProgram(shaderProgram);

	// создадим перспективную матрицу
	Matrix4Perspective(projectionMatrix, 45.0f,
		(float)window->width / (float)window->height, 0.5f, 5.0f);

	projectionMatrixLocation = glGetUniformLocation(shaderProgram, "projectionMatrix");

	if (projectionMatrixLocation != -1)
		glUniformMatrix4fv(projectionMatrixLocation, 1, GL_TRUE, projectionMatrix);

	// проверка на корректность шейдерной программы
	glValidateProgram(shaderProgram);
	if (ShaderProgramStatus(shaderProgram, GL_VALIDATE_STATUS) != GL_TRUE)
		return false;

	// проверим не было ли ошибок
	OPENGL_CHECK_FOR_ERRORS();

	// создадим и используем Vertex Array Object (VAO)
	glGenVertexArrays(1, &meshVAO);
	glBindVertexArray(meshVAO);

	// создадим и используем Vertex Buffer Object (VBO)
	glGenBuffers(1, &meshVBO);
	glBindBuffer(GL_ARRAY_BUFFER, meshVBO);

	// заполним VBO данными треугольника
	glBufferData(GL_ARRAY_BUFFER, MESH_VERTEX_COUNT * VERTEX_SIZE,
		pointsMesh, GL_STATIC_DRAW);

	// получим позицию атрибута 'position' из шейдера
	positionLocation = glGetAttribLocation(shaderProgram, "position");
	if (positionLocation != -1)
	{
		// назначим на атрибут параметры доступа к VBO
		glVertexAttribPointer(positionLocation, 3, GL_FLOAT, GL_FALSE,
			VERTEX_SIZE, (const GLvoid*)VERTEX_POSITION_OFFSET);
		// разрешим использование атрибута
		glEnableVertexAttribArray(positionLocation);
	}

	// получим позицию атрибута 'color' из шейдера
	colorLocation = glGetAttribLocation(shaderProgram, "color");
	if (colorLocation != -1)
	{
		// назначим на атрибут параметры доступа к VBO
		glVertexAttribPointer(colorLocation, 3, GL_FLOAT, GL_FALSE,
			VERTEX_SIZE, (const GLvoid*)VERTEX_COLOR_OFFSET);
		// разрешим использование атрибута
		glEnableVertexAttribArray(colorLocation);
	}

	OPENGL_CHECK_FOR_ERRORS();

	return true;
}

// очистка OpenGL
void GLWindowClear(const GLWindow *window)
{
	ASSERT(window);

	// удаляем VAO и VBO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &meshVBO);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, &meshVAO);

	glUseProgram(0);
	glDeleteProgram(shaderProgram);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

// функция рендера
void GLWindowRender(const GLWindow *window)
{
	ASSERT(window);

	// очистим буфер цвета и глубины
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// делаем шейдерную программу активной
	glUseProgram(shaderProgram);


	// для рендеринга исопльзуем VAO
	glBindVertexArray(meshVAO);

	//задаем размер точки
	glPointSize(7);
	// рендер точек из VBO привязанного к VAO
	glDrawArrays(GL_POINTS, 0, MESH_VERTEX_COUNT);

	// проверка на ошибки
	OPENGL_CHECK_FOR_ERRORS();
}

// функция обновления
void GLWindowUpdate(const GLWindow *window, double deltaTime)
{
	ASSERT(window);
	ASSERT(deltaTime >= 0.0); // проверка на возможность бага
}

// функция обработки ввода с клавиатуры и мыши
void GLWindowInput(const GLWindow *window)
{
	ASSERT(window);

	// выход из приложения по кнопке Esc
	if (InputIsKeyPressed(VK_ESCAPE))
		GLWindowDestroy();

	// переключение между оконным и полноэкранным режимом
	// осуществляется по нажатию комбинации Alt+Enter
	if (InputIsKeyDown(VK_MENU) && InputIsKeyPressed(VK_RETURN))
		GLWindowSetSize(window->width, window->height, !window->fullScreen);
}

static double GetTimerTicks()
{
	QueryPerformanceCounter(&g_qpc);
	return g_timerFrequency * g_qpc.QuadPart;
}

bool GLWindowCreate(const char *title, int width, int height, bool fullScreen)
{
	ASSERT(title);
	ASSERT(width > 0);
	ASSERT(height > 0);

	WNDCLASSEX            wcx;
	PIXELFORMATDESCRIPTOR pfd;
	RECT                  rect;
	HGLRC                 hRCTemp;
	DWORD                 style, exStyle;
	int                   x, y, format;

	// обнуляем стейт окна
	memset(&g_window, 0, sizeof(g_window));

	// обнуляем стейт ввода
	memset(&g_input, 0, sizeof(g_input));

	// определим указатель на функцию создания расширенного контекста OpenGL
	PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB = NULL;

	// укажем атрибуты для создания расширенного контекста OpenGL
	int attribs[] =
	{
		WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
		WGL_CONTEXT_MINOR_VERSION_ARB, 3,
		WGL_CONTEXT_FLAGS_ARB,         WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
		WGL_CONTEXT_PROFILE_MASK_ARB,  WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
		0
	};

	// инциализация таймера
	QueryPerformanceFrequency(&g_qpc);
	ASSERT(g_qpc.QuadPart > 0);

	g_timerFrequency = 1.0 / g_qpc.QuadPart;

	g_hInstance = (HINSTANCE)GetModuleHandle(NULL);

	// регистрация класса окна
	memset(&wcx, 0, sizeof(wcx));
	wcx.cbSize        = sizeof(wcx);
	wcx.style         = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wcx.lpfnWndProc   = (WNDPROC)GLWindowProc;
	wcx.hInstance     = g_hInstance;
	wcx.lpszClassName = GLWINDOW_CLASS_NAME;
	wcx.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
	wcx.hCursor       = LoadCursor(NULL, IDC_ARROW);

	if (!RegisterClassEx(&wcx))
	{
		LOG_ERROR("RegisterClassEx fail (%d)\n", GetLastError());
		return false;
	}

	// стили окна
	style   = WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
	exStyle = WS_EX_APPWINDOW;

	// выровняем окно по центру экрана
	x = (GetSystemMetrics(SM_CXSCREEN) - width)  / 2;
	y = (GetSystemMetrics(SM_CYSCREEN) - height) / 2;

	rect.left   = x;
	rect.right  = x + width;
	rect.top    = y;
	rect.bottom = y + height;

	// подгоним размер окна под стили
	AdjustWindowRectEx (&rect, style, FALSE, exStyle);

	// создаем окно
	g_hWnd = CreateWindowEx(exStyle, GLWINDOW_CLASS_NAME, title, style, rect.left, rect.top,
		rect.right - rect.left, rect.bottom - rect.top, NULL, NULL, g_hInstance, NULL);

	if (!g_hWnd)
	{
		LOG_ERROR("CreateWindowEx fail (%d)\n", GetLastError());
		return false;
	}

	// получим дескриптор контекста окна
	g_hDC = GetDC(g_hWnd);

	if (!g_hDC)
	{
		LOG_ERROR("GetDC fail (%d)\n", GetLastError());
		return false;
	}

	// описание формата пикселей
	memset(&pfd, 0, sizeof(pfd));
	pfd.nSize      = sizeof(pfd);
	pfd.nVersion   = 1;
	pfd.dwFlags    = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 32;
	pfd.cDepthBits = 24;

	// запросим формат пикселей, ближайший к описанному выше
	format = ChoosePixelFormat(g_hDC, &pfd);
	if (!format || !SetPixelFormat(g_hDC, format, &pfd))
	{
		LOG_ERROR("Setting pixel format fail (%d)\n", GetLastError());
		return false;
	}

	// создадим временный контекст рендеринга
	// он нужен для получения функции wglCreateContextAttribsARB
	hRCTemp = wglCreateContext(g_hDC);
	if (!hRCTemp || !wglMakeCurrent(g_hDC, hRCTemp))
	{
		LOG_ERROR("Сreating temp render context fail (%d)\n", GetLastError());
		return false;
	}

	// получим адрес функции установки атрибутов контекста рендеринга
	OPENGL_GET_PROC(PFNWGLCREATECONTEXTATTRIBSARBPROC, wglCreateContextAttribsARB);

	// временный контекст OpenGL нам больше не нужен
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRCTemp);

	// создадим расширенный контекст с поддержкой OpenGL 3
	g_hRC = wglCreateContextAttribsARB(g_hDC, 0, attribs);
	if (!g_hRC || !wglMakeCurrent(g_hDC, g_hRC))
	{
		LOG_ERROR("Creating render context fail (%d)\n", GetLastError());
		return false;
	}

	// выведем в лог немного информации о контексте OpenGL
	int major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);
	LOG_DEBUG("OpenGL render context information:\n"
		"  Renderer       : %s\n"
		"  Vendor         : %s\n"
		"  Version        : %s\n"
		"  GLSL version   : %s\n"
		"  OpenGL version : %d.%d\n",
		(const char*)glGetString(GL_RENDERER),
		(const char*)glGetString(GL_VENDOR),
		(const char*)glGetString(GL_VERSION),
		(const char*)glGetString(GL_SHADING_LANGUAGE_VERSION),
		major, minor
	);

	// попробуем загрузить расширения OpenGL
	if (!OpenGLInitExtensions())
		return false;

	// зададим размеры окна
	GLWindowSetSize(width, height, fullScreen);

	return true;
}

void GLWindowDestroy()
{
	g_window.running = g_window.active = false;

	GLWindowClear(&g_window);

	// восстановим разрешение экрана
	if (g_window.fullScreen)
	{
		ChangeDisplaySettings(NULL, CDS_RESET);
		ShowCursor(TRUE);
		g_window.fullScreen = false;
	}

	// удаляем контекст рендеринга
	if (g_hRC)
	{
		wglMakeCurrent(NULL, NULL);
		wglDeleteContext(g_hRC);
		g_hRC = NULL;
	}

	// освобождаем контекст окна
	if (g_hDC)
	{
		ReleaseDC(g_hWnd, g_hDC);
		g_hDC = NULL;
	}

	// удаляем окно
	if (g_hWnd)
	{
		DestroyWindow(g_hWnd);
		g_hWnd = NULL;
	}

	// удаляем класс окна
	if (g_hInstance)
	{
		UnregisterClass(GLWINDOW_CLASS_NAME, g_hInstance);
		g_hInstance = NULL;
	}
}

void GLWindowSetSize(int width, int height, bool fullScreen)
{
	ASSERT(width > 0);
	ASSERT(height > 0);

	RECT    rect;
	DWORD   style, exStyle;
	DEVMODE devMode;
	LONG    result;
	int     x, y;

	// если мы возвращаемся из полноэкранного режима
	if (g_window.fullScreen && !fullScreen)
	{
		ChangeDisplaySettings(NULL, CDS_RESET);
		ShowCursor(TRUE);
	}

	g_window.fullScreen = fullScreen;

	// если необходим полноэкранный режим
	if (g_window.fullScreen)
	{
		memset(&devMode, 0, sizeof(devMode));
		devMode.dmSize       = sizeof(devMode);
		devMode.dmPelsWidth  = width;
		devMode.dmPelsHeight = height;
		devMode.dmBitsPerPel = GetDeviceCaps(g_hDC, BITSPIXEL);
		devMode.dmFields     = DM_PELSWIDTH | DM_PELSHEIGHT | DM_BITSPERPEL;

		// попытка установить полноэкранный режим
		result = ChangeDisplaySettings(&devMode, CDS_FULLSCREEN);
		if (result != DISP_CHANGE_SUCCESSFUL)
		{
			LOG_ERROR("ChangeDisplaySettings fail %dx%d (%d)\n", width, height, result);
			g_window.fullScreen = false;
		}
	}

	// если был запрошен полноэкранный режим и его удалось установить
	if (g_window.fullScreen)
	{
		ShowCursor(FALSE);

		style   = WS_POPUP;
		exStyle = WS_EX_APPWINDOW | WS_EX_TOPMOST;

		x = y = 0;
	} else // если полноэкранный режим не нужен, или его не удалось установить
	{
		style   = WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
		exStyle = WS_EX_APPWINDOW;

		// выровняем окно по центру экрана
		x = (GetSystemMetrics(SM_CXSCREEN) - width)  / 2;
		y = (GetSystemMetrics(SM_CYSCREEN) - height) / 2;
	}

	rect.left   = x;
	rect.right  = x + width;
	rect.top    = y;
	rect.bottom = y + height;

	// подгоним размер окна под стили
	AdjustWindowRectEx (&rect, style, FALSE, exStyle);

	// установим стили окна
	SetWindowLong(g_hWnd, GWL_STYLE,   style);
	SetWindowLong(g_hWnd, GWL_EXSTYLE, exStyle);

	// обновим позицию окна
	SetWindowPos(g_hWnd, HWND_TOP, rect.left, rect.top,
		rect.right - rect.left, rect.bottom - rect.top,
		SWP_FRAMECHANGED);

	// покажем окно на экране
	ShowWindow(g_hWnd, SW_SHOW);
	SetForegroundWindow(g_hWnd);
	SetFocus(g_hWnd);
	UpdateWindow(g_hWnd);

	// получим размеры окна
	GetClientRect(g_hWnd, &rect);
	g_window.width  = rect.right - rect.left;
	g_window.height = rect.bottom - rect.top;

	// центрируем курсор относительно окна
	InputSetCursorPos(g_window.width / 2, g_window.height / 2);

	OPENGL_CHECK_FOR_ERRORS();
}

int GLWindowMainLoop()
{
	MSG    msg;
	double beginFrameTime, deltaTime;

	float rotate = 0.;

	// основной цикл окна
	g_window.running = g_window.active = GLWindowInit(&g_window);

	while (g_window.running)
	{
		// обработаем сообщения из очереди сообщений
		while (PeekMessage(&msg, g_hWnd, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
			{
				g_window.running = false;
				break;
			}
			// TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		GLWindowInput(&g_window);

		// rotation about X axis
		glRotatef(0, 1.0, 0.0, 0.0);
		// rotation about Y axis
		glRotatef(0, 0.0, 1.0, 0.0);
		// rotation about Z axis
		glRotatef(rotate, 0.0, 0.0, 1.0);

		// если окно в рабочем режиме и активно
		if (g_window.running && g_window.active)
		{
			// надо брать время из таймера
			beginFrameTime = GetTimerTicks();

			GLWindowRender(&g_window);
			SwapBuffers(g_hDC);

			// надо вычитать из текущего значения таймера
			deltaTime = GetTimerTicks() - beginFrameTime;

			GLWindowUpdate(&g_window, deltaTime);
		}

		Sleep(2);
	}

	g_window.running = g_window.active = false;
	return 0;
}

LRESULT CALLBACK GLWindowProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
		case WM_LBUTTONDOWN:
		case WM_LBUTTONUP:
		case WM_RBUTTONDOWN:
		case WM_RBUTTONUP:
		case WM_MBUTTONDOWN:
		case WM_MBUTTONUP:
		{
			g_input.cursorPos[0] = (int16_t)LOWORD(lParam);
			g_input.cursorPos[1] = (int16_t)HIWORD(lParam);

			if (msg == WM_LBUTTONDOWN || msg == WM_LBUTTONUP)
				g_input.buttonState[0] = (msg == WM_LBUTTONDOWN ? INPUT_PRESSED : INPUT_UP);

			if (msg == WM_RBUTTONDOWN || msg == WM_RBUTTONUP)
				g_input.buttonState[1] = (msg == WM_RBUTTONDOWN ? INPUT_PRESSED : INPUT_UP);

			if (msg == WM_MBUTTONDOWN || msg == WM_MBUTTONUP)
				g_input.buttonState[2] = (msg == WM_MBUTTONDOWN ? INPUT_PRESSED : INPUT_UP);

			return FALSE;
		}

		case WM_MOUSEMOVE:
		{
			g_input.cursorPos[0] = (int16_t)LOWORD(lParam);
			g_input.cursorPos[1] = (int16_t)HIWORD(lParam);

			return FALSE;
		}

		case WM_KEYDOWN:
		case WM_SYSKEYDOWN:
		{
			if (wParam < 256 && (lParam & 0x40000000) == 0)
				g_input.keyState[wParam] = INPUT_PRESSED;

			return FALSE;
		}

		case WM_KEYUP:
		case WM_SYSKEYUP:
		{
			if (wParam < 256)
				g_input.keyState[wParam] = INPUT_UP;

			return FALSE;
		}

		case WM_SETFOCUS:
		case WM_KILLFOCUS:
		{
			g_window.active = (msg == WM_SETFOCUS);
			return FALSE;
		}

		case WM_ACTIVATE:
		{
			g_window.active = (LOWORD(wParam) != WA_INACTIVE);
			return FALSE;
		}

		case WM_CLOSE:
		{
			g_window.running = g_window.active = false;
			PostQuitMessage(0);
			return FALSE;
		}

		case WM_SYSCOMMAND:
		{
			switch (wParam & 0xFFF0)
			{
				case SC_SCREENSAVE:
				case SC_MONITORPOWER:
				{
					if (g_window.fullScreen)
						return FALSE;

					break;
				}

				case SC_KEYMENU:
				case SC_TASKLIST:
				{
					return FALSE;
				}
			}

			break;
		}

		case WM_ERASEBKGND:
		{
			return FALSE;
		}
	}

	return DefWindowProc(hWnd, msg, wParam, lParam);
}

bool InputIsKeyDown(uint8_t key)
{
	return (g_input.keyState[key] != 0);
}

bool InputIsKeyPressed(uint8_t key)
{
	bool pressed = (g_input.keyState[key] == INPUT_PRESSED);
	g_input.keyState[key] = INPUT_DOWN;
	return pressed;
}

bool InputIsButtonDown(uint8_t button)
{
	ASSERT(button < 3);

	return (g_input.buttonState[button] != 0);
}

bool InputIsButtonClick(uint8_t button)
{
	ASSERT(button < 3);

	bool pressed = (g_input.buttonState[button] == INPUT_PRESSED);
	g_input.buttonState[button] = INPUT_DOWN;
	return pressed;
}

void InputGetCursorPos(int16_t *x, int16_t *y)
{
	ASSERT(x);
	ASSERT(y);

	*x = g_input.cursorPos[0];
	*y = g_input.cursorPos[1];
}

void InputSetCursorPos(int16_t x, int16_t y)
{
	POINT pos = {x, y};
	ClientToScreen(g_hWnd, &pos);
	SetCursorPos(pos.x, pos.y);

	g_input.cursorPos[0] = x;
	g_input.cursorPos[1] = y;
}

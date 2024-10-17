#include "Graphics.h"
#include <cmath>
#include <iostream>
#include "physics/Vec2.h"

int Graphics::m_windowWidth = 1280;
int Graphics::m_windowHeight = 720;

int Graphics::Width()
{
	return m_windowWidth;
}

int Graphics::Height()
{
	return m_windowHeight;
}

void Graphics::OpenWindow()
{
	InitWindow(m_windowWidth, m_windowHeight, "Physics 2D");

	/*const int currentMonitorId = GetCurrentMonitor();
	m_windowWidth = GetMonitorWidth(currentMonitorId);
	m_windowHeight = GetMonitorHeight(currentMonitorId);

	SetWindowSize(m_windowWidth, m_windowHeight);
	SetWindowState(FLAG_WINDOW_RESIZABLE);*/
}

void Graphics::ClearScreen(const Color &color)
{
	ClearBackground(color);
}

void Graphics::DrawLine(const Vec2 &startPos, const Vec2 &endPos, const Color &color)
{
	Vector2 start;
	start.x = startPos.x;
	start.y = startPos.y;

	Vector2 end;
	end.x = endPos.x;
	end.y = endPos.y;

	DrawLineV(start, end, color);
}

void Graphics::DrawCircle(const Vec2 &pos, const float radius, const float angle, const Color &color)
{
	RLAPI::DrawCircleLines(static_cast<int>(pos.x), static_cast<int>(pos.y), radius, color);

	Vector2 center;
	center.x = pos.x;
	center.y = pos.y;

	Vector2 end;
	end.x = pos.x + cos(angle) * radius;
	end.y = pos.y + sin(angle) * radius;

	DrawLineV(center, end, color);
}

void Graphics::DrawFillCircle(const Vec2 &pos, const float radius, const Color &color)
{
	Vector2 center;
	center.x = pos.x;
	center.y = pos.y;

	RLAPI::DrawCircleV(center, radius, color);
}

void Graphics::DrawRect(const Vec2 &pos, const int width, const int height, const Color &color)
{
	const int centerX = static_cast<int>(pos.x - static_cast<float>(width) / 2);
	const int centerY = static_cast<int>(pos.y - static_cast<float>(height) / 2);
	RLAPI::DrawRectangleLines(centerX, centerY, width, height, color);
}

void Graphics::DrawFillRect(const Vec2 &pos, const int width, const int height, const Color &color)
{
	const int centerX = static_cast<int>(pos.x - static_cast<float>(width) / 2);
	const int centerY = static_cast<int>(pos.y - static_cast<float>(height) / 2);
	RLAPI::DrawRectangle(centerX, centerY, width, height, color);
}

void Graphics::DrawPolygon(const Vec2 &pos, const std::vector<Vec2> &vertices, const Color &color)
{
	for (size_t i = 0; i < vertices.size(); i++)
	{
		const auto currIndex = i;
		const auto nextIndex = (i + 1) % vertices.size();
		RLAPI::DrawLine(
			 static_cast<int>(vertices[currIndex].x), static_cast<int>(vertices[currIndex].y), 
			static_cast<int>(vertices[nextIndex].x), static_cast<int>(vertices[nextIndex].y), color);
	}

	RLAPI::DrawCircle(static_cast<int>(pos.x), static_cast<int>(pos.y), 1.0f, color);
}

void Graphics::DrawFillPolygon(const Vec2 &center, const std::vector<Vec2> &vertices, const Color &color)
{
	for (size_t i = 2; i < vertices.size(); ++i)
	{
		const auto v0 = vertices[0];
		const auto v1 = vertices[i];
		const auto v2 = vertices[i - 1];

		DrawTriangle(Vector2{v0.x, v0.y},Vector2{v1.x, v1.y}, Vector2{v2.x, v2.y}, color);
	}

	RLAPI::DrawCircle(static_cast<int>(center.x), static_cast<int>(center.y), 1.0f, BLACK);
}

void Graphics::DrawTexture(const Vec2 &pos, const float width, const float height, const float rotation, const Texture2D& texture)
{
	const Rectangle srcRect = {0, 0, static_cast<float>(texture.width), static_cast<float>(texture.height)};
	const Rectangle destRect = {pos.x, pos.y, width, height};

	const float rotationDeg = rotation * 57.2958F;

	Vector2 origin;
	origin.x = width / 2;
	origin.y = height/ 2;

	RLAPI::DrawTexturePro(texture, srcRect, destRect, origin, rotationDeg, WHITE);
}

void Graphics::DrawText(const std::string& text, const Color& color)
{
	DrawText(text, color);
}

void Graphics::CloseWindow()
{
	RLAPI::CloseWindow();
}

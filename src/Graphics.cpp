#include "Graphics.h"
#include "physics/Vec2.h"
#include <iostream>
#include <cmath>

int Graphics::windowWidth = 1280;
int Graphics::windowHeight = 720;

int Graphics::Width()
{
	return windowWidth;
}

int Graphics::Height()
{
	return windowHeight;
}

void Graphics::OpenWindow()
{
	InitWindow(windowWidth, windowHeight, "Physics 2D");
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

void Graphics::DrawCircle(const Vec2 &pos, int radius, float angle, const Color &color)
{
	DrawCircleLines(pos.x, pos.y, radius, color);

	Vector2 center;
	center.x = pos.x;
	center.y = pos.y;

	Vector2 end;
	end.x = pos.x + cos(angle) * radius;
	end.y = pos.y + sin(angle) * radius;

	DrawLineV(center, end, color);
}

void Graphics::DrawFillCircle(const Vec2 &pos, int radius, const Color &color)
{
	Vector2 center;
	center.x = pos.x;
	center.y = pos.y;

	RLAPI::DrawCircleV(center, radius, color);
}

void Graphics::DrawRect(const Vec2 &pos, int width, int height, const Color &color)
{
	DrawRectangleLines(pos.x, pos.y, width, height, color);
}

void Graphics::DrawFillRect(const Vec2 &pos, int width, int height, const Color &color)
{
	DrawRectangle(pos.x, pos.y, width, height, color);
}

void Graphics::DrawPolygon(const Vec2 &pos, const std::vector<Vec2> &vertices, const Color &color)
{
	for (size_t i = 0; i < vertices.size(); i++)
	{
		const int curr_index = i;
		const int next_index = (i + 1) % vertices.size();
		RLAPI::DrawLine(vertices[curr_index].x, vertices[curr_index].y, vertices[next_index].x, vertices[next_index].y, color);
	}

	RLAPI::DrawCircle(pos.x, pos.y, 1, color);
}

void Graphics::DrawFillPolygon(const Vec2 &pos, const std::vector<Vec2> &vertices, const Color &color)
{
	std::vector<Vector2> rayVertices;

	for (const auto &vertice : vertices)
	{
		Vector2 rayVertice;
		rayVertice.x = vertice.x;
		rayVertice.y = vertice.y;
		rayVertices.push_back(rayVertice);
	}

	DrawTriangleStrip(rayVertices.data(), rayVertices.size(), color);
	RLAPI::DrawCircle(pos.x, pos.y, 1, BLACK);
}

void Graphics::DrawTexture(const Vec2 &pos, int width, int height, float rotation, Texture2D *texture)
{
	const Rectangle srcRect = {0, 0, texture->width, texture->height};
	const Rectangle destRect = {pos.x - (width / 2), pos.y - (height / 2), width, height};

	const float rotation_deg = rotation * 57.2958F;

	Vector2 origin;
	origin.x = pos.x;
	origin.y = pos.y;

	DrawTexturePro(*texture, srcRect, destRect, origin, rotation_deg, BLANK);
}

void Graphics::CloseWindow(void)
{
	RLAPI::CloseWindow();
}
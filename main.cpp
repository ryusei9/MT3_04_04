#include <Novice.h>
#include <Vector3.h>
#include <Matrix4x4.h>
#include <cstdint>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>
#include <imgui.h>
#include <numbers>
#include <algorithm>

const char kWindowTitle[] = "LE2B_13_サトウ_リュウセイ_MT3_04_01";

struct Line {
	// 始点
	Vector3 origin;
	// 終点への差分ベクトル
	Vector3 diff;
};

struct Ray {
	// 始点
	Vector3 origin;
	// 終点への差分ベクトル
	Vector3 diff;
};

struct Segment {
	// 始点
	Vector3 origin;
	// 終点への差分ベクトル
	Vector3 diff;
	unsigned int color;
};

struct Sphere {
	Vector3 center;
	float radius;
	unsigned int color;
};

struct Plane {
	Vector3 normal;
	float distance;
	unsigned int color;
};

struct AABB {
	Vector3 min;
	Vector3 max;
	unsigned int color;
};

struct Spring {
	// アンカー
	Vector3 anchor;
	// 自然長
	float naturalLength;
	// 剛性。ばね定数k
	float stiffness;
	// 減衰係数
	float dampingCoefficient;
};

struct Ball {
	Vector3 position;
	Vector3 velocity;
	Vector3 acceleration;
	float mass;
	float radius;
	float angularVelocity;
	float angle;
	unsigned int color;
};

struct Pendulum {
	// アンカーポイント
	Vector3 anchor;
	// 紐の長さ
	float length;
	// 現在の角度
	float angle;
	// 角速度ω
	float anglarVelocity;
	// 各加速度
	float anglarAcceleration;
};

struct ConicalPendulum {
	// アンカーポイント
	Vector3 anchor;
	// 紐の長さ
	float length;
	// 円錐の頂点の半分
	float halfApexAngle;
	// 現在の角度
	float angle;
	// 角速度ω
	float anglarVelocity;
};

const float kWindowWidth = 1280.0f;
const float kWindowHeight = 720.0f;

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix);

void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);

void DrawAABB(const AABB& a, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);

void DrawHook(const Spring& s, const Ball& b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);
// 行列の積
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2);
// 逆行列
Matrix4x4 Inverse(const Matrix4x4& m);
// 座標変換
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix);
// 3次元アフィン変換行列
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3 translate);

// x軸回転行列
Matrix4x4 MakeRotateXMatrix(float radian);

// y軸回転行列
Matrix4x4 MakeRotateYMatrix(float radian);

// z軸回転行列
Matrix4x4 MakeRotateZMatrix(float radian);

// 透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip);

// ビューポート変換行列
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth);

// 加算
Vector3 Add(const Vector3& v1, const Vector3& v2);
// 減算
Vector3 Subtract(const Vector3& v1, const Vector3& v2);

Vector3 Multiply(const Vector3& v1, const Vector3& v2);

// 行列の加法
Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2);
// 行列の減法
Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2);

// スカラー倍
Vector3 Multiply(float scalar, const Vector3& v);
// 内積
float Dot(const Vector3& v1, const Vector3& v2);
// 長さ
float Length(const Vector3& v);
// 正規化
Vector3 Normalize(const Vector3 v);
// 正射影ベクトル
Vector3 Project(const Vector3& v1, const Vector3& v2);
// 最近接点
Vector3 ClosestPoint(const Vector3& point, const Segment& segment);

void DrawSegment(const Segment& segment, const Matrix4x4& worldViewProjectionMatrix, const Matrix4x4& viewPortMatrix);

bool IsCollision(const Sphere& s1, const Sphere& s2);

bool IsCollision(const Sphere& s, const Plane& p);

bool IsCollision(const Plane& p, const Segment& s);

bool IsCollision(const AABB& aabb, const AABB& aabb2);

bool IsCollision(const AABB& aabb, const Sphere& sphere);

bool IsCollision(const AABB& aabb, const Segment& segment);

// 垂直
Vector3 Perpendicular(const Vector3& v);

// 平面描画
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);

// ばねの運動
void Hook(Spring& s, Ball& b);

// 円運動
void circularMotion(Ball& b, Vector3 c);

// 円の描画
void DrawEllipse(Ball b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);

// 振り子の運動
void PendulumSwing(Pendulum& p, Ball& b);

// 振り子の描画
void DrawPendulum(Pendulum p, Ball b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);

// 円錐振り子の運動
void ConicalPendulumSwing(ConicalPendulum& p, Ball& b);

// 円錐振り子の描画
void DrawConicalPendulum(ConicalPendulum p, Ball b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix);

// 反射ベクトルを求める関数
Vector3 Reflect(const Vector3& input, const Vector3& normal);

// 反発
void BallReflect(Ball& b, Plane p);

// 演算子オーバーロード
Vector3 operator+(const Vector3& v1, const Vector3& v2) { return Add(v1, v2); }
Vector3 operator-(const Vector3& v1, const Vector3& v2) { return Subtract(v1, v2); }
Vector3 operator*(const Vector3& v1, const Vector3& v2) { return Multiply(v1, v2); }
Vector3 operator*(float s, const Vector3& v) { return Multiply(s, v); }
Vector3 operator*(const Vector3& v, float s) { return s * v; }
Vector3 operator/(const Vector3& v, float s) { return Multiply(1.0f / s, v); }
Vector3 operator-(const Vector3& v) { return { -v.x, -v.y, -v.z }; }
Vector3 operator+(const Vector3& v) { return v; }
Matrix4x4 operator+(const Matrix4x4& m1, const Matrix4x4& m2) { return Add(m1, m2); }
Matrix4x4 operator-(const Matrix4x4& m1, const Matrix4x4& m2) { return Subtract(m1, m2); }
Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2) { return Multiply(m1, m2); }

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Vector3 rotate = { 0.26f,0.0f,0.0f };

	Vector3 cameraTranslate{ 0.0f,0.0f,-10.0f };
	Vector3 cameraRotate{ 0.0f,0.0f,0.0f };

	// ボール
	Ball ball{};
	ball.position = { 0.8f,1.2f,0.3f };
	ball.acceleration.y = -9.8f;
	ball.mass = 2.0f;
	ball.radius = 0.05f;
	// 角速度
	ball.angularVelocity = 3.14f;
	// θ
	ball.angle = 0.0f;
	ball.color = WHITE;

	// 平面
	Plane plane = { Normalize({-0.2f,0.9f,-0.3f}),0.0f,{0xFFFFFFFF}};

	// 反射のフラグ
	bool isReflectStart = false;

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///
		ImGui::DragFloat3("CameraTranslate", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("WorldRotate", &rotate.x, 0.01f);
		ImGui::Checkbox("Start", &isReflectStart);

		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, { 0,0,0 });
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 worldViewMatrix = Inverse(worldMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 worldViewProjectionMatrix = Multiply(worldViewMatrix, Multiply(viewMatrix, projectionMatrix));
		Matrix4x4 viewPortMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		// 反射開始
		if (isReflectStart) {
			BallReflect(ball, plane);
		}
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		DrawGrid(worldViewProjectionMatrix, viewPortMatrix);
		DrawPlane(plane, worldViewProjectionMatrix, viewPortMatrix);
		DrawEllipse(ball, worldViewProjectionMatrix, viewPortMatrix);
		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
// 球の描画
void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {

	const uint32_t kSubdivision = 10; // 分割数
	const float kLatEvery = (float)M_PI / kSubdivision; // 経度分割1つ分の角度
	const float kLonEvery = (2 * (float)M_PI) / kSubdivision; // 緯度分割1つ分の角度

	// 緯度の方向に分割 -Π/2 ~ Π/2
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {

		float lat = -(float)M_PI / 2.0f + latIndex * kLatEvery; // 現在の緯度

		// 経度の方向に分割 0 ~ 2Π
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {

			float lon = lonIndex * kLonEvery; // 現在の経度

			// World座標系でのa、b、cを求める
			Vector3 a, b, c;

			a = {
				sphere.center.x + sphere.radius * cosf(lat) * cosf(lon),
				sphere.center.y + sphere.radius * sinf(lat),
				sphere.center.z + sphere.radius * cosf(lat) * sinf(lon)
			};

			b = {
				sphere.center.x + sphere.radius * cosf(lat + kLatEvery) * cosf(lon),
				sphere.center.y + sphere.radius * sinf(lat + kLatEvery),
				sphere.center.z + sphere.radius * cosf(lat + kLatEvery) * sinf(lon)
			};

			c = {
				sphere.center.x + sphere.radius * cosf(lat) * cosf(lon + kLonEvery),
				sphere.center.y + sphere.radius * sinf(lat),
				sphere.center.z + sphere.radius * cosf(lat) * sinf(lon + kLonEvery)
			};

			// a、b、cをScreen座標系まで変換
			a = Transform(a, Multiply(viewProjectionMatrix, viewportMatrix));
			b = Transform(b, Multiply(viewProjectionMatrix, viewportMatrix));
			c = Transform(c, Multiply(viewProjectionMatrix, viewportMatrix));

			// ab、bcで線を引く
			Novice::DrawLine(int(a.x), int(a.y), int(b.x), int(b.y), sphere.color);
			Novice::DrawLine(int(a.x), int(a.y), int(c.x), int(c.y), sphere.color);
		}
	}
}

void DrawAABB(const AABB& a, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix) {

	Vector3 vertices[8]{};
	vertices[0] = { a.min.x,a.min.y,a.min.z };
	vertices[1] = { a.max.x,a.min.y,a.min.z };
	vertices[2] = { a.min.x,a.max.y,a.min.z };
	vertices[3] = { a.max.x,a.max.y,a.min.z };
	vertices[4] = { a.min.x,a.min.y,a.max.z };
	vertices[5] = { a.max.x,a.min.y,a.max.z };
	vertices[6] = { a.min.x,a.max.y,a.max.z };
	vertices[7] = { a.max.x,a.max.y,a.max.z };
	for (uint32_t i = 0; i < 8; i++) {
		vertices[i] = Transform(vertices[i], Multiply(viewProjectionMatrix, viewPortMatrix));
	}

	Novice::DrawLine(int(vertices[0].x), int(vertices[0].y), int(vertices[1].x), int(vertices[1].y), a.color);
	Novice::DrawLine(int(vertices[0].x), int(vertices[0].y), int(vertices[2].x), int(vertices[2].y), a.color);
	Novice::DrawLine(int(vertices[0].x), int(vertices[0].y), int(vertices[4].x), int(vertices[4].y), a.color);
	Novice::DrawLine(int(vertices[1].x), int(vertices[1].y), int(vertices[3].x), int(vertices[3].y), a.color);
	Novice::DrawLine(int(vertices[1].x), int(vertices[1].y), int(vertices[5].x), int(vertices[5].y), a.color);
	Novice::DrawLine(int(vertices[2].x), int(vertices[2].y), int(vertices[6].x), int(vertices[6].y), a.color);
	Novice::DrawLine(int(vertices[2].x), int(vertices[2].y), int(vertices[3].x), int(vertices[3].y), a.color);
	Novice::DrawLine(int(vertices[3].x), int(vertices[3].y), int(vertices[7].x), int(vertices[7].y), a.color);
	Novice::DrawLine(int(vertices[4].x), int(vertices[4].y), int(vertices[5].x), int(vertices[5].y), a.color);
	Novice::DrawLine(int(vertices[4].x), int(vertices[4].y), int(vertices[6].x), int(vertices[6].y), a.color);
	Novice::DrawLine(int(vertices[5].x), int(vertices[5].y), int(vertices[7].x), int(vertices[7].y), a.color);
	Novice::DrawLine(int(vertices[6].x), int(vertices[6].y), int(vertices[7].x), int(vertices[7].y), a.color);
}

void DrawHook(const Spring& s, const Ball& b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix) {
	Vector3 start = Transform(s.anchor, Multiply(viewProjectionMatrix, viewPortMatrix));
	Vector3 ball = Transform(b.position, Multiply(viewProjectionMatrix, viewPortMatrix));
	Novice::DrawLine(int(start.x), int(start.y), int(ball.x), int(ball.y), 0x000000FF);
	Novice::DrawEllipse(int(ball.x), int(ball.y), int(b.radius), int(b.radius), 0.0f, b.color, kFillModeSolid);
}

// グリッドの描画
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {

	const float kGridHalfWidth = 2.0f;                                      // Gridの半分の幅
	const uint32_t kSubdivision = 10;                                       // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision); // 1つ分の長さ

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {

		float posX = -kGridHalfWidth + kGridEvery * xIndex;

		Vector3 start = { posX, 0.0f, -kGridHalfWidth };
		Vector3 end = { posX, 0.0f, kGridHalfWidth };

		start = Transform(start, Multiply(viewProjectionMatrix, viewportMatrix));
		end = Transform(end, Multiply(viewProjectionMatrix, viewportMatrix));

		// 左から右も同じように順々に引いていく
		for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
			float posZ = -kGridHalfWidth + kGridEvery * zIndex;

			Vector3 startZ = { -kGridHalfWidth, 0.0f, posZ };
			Vector3 endZ = { kGridHalfWidth, 0.0f, posZ };

			startZ = Transform(startZ, Multiply(viewProjectionMatrix, viewportMatrix));
			endZ = Transform(endZ, Multiply(viewProjectionMatrix, viewportMatrix));

			Novice::DrawLine((int)start.x, (int)start.y, (int)end.x, (int)end.y, 0xFFFFFFFF);
			Novice::DrawLine((int)startZ.x, (int)startZ.y, (int)endZ.x, (int)endZ.y, 0xFFFFFFFF);
		}
	}
}

// 行列の積
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 resultMultiply = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			resultMultiply.m[i][j] = m1.m[i][0] * m2.m[0][j] + m1.m[i][1] * m2.m[1][j] + m1.m[i][2] * m2.m[2][j] + m1.m[i][3] * m2.m[3][j];
		}
	}
	return resultMultiply;
}
// 逆行列
Matrix4x4 Inverse(const Matrix4x4& m) {
	Matrix4x4 resultInverse = {};
	float A = m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3] + m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1] + m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2]
		- m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1] - m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3] - m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2]
		- m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3] - m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1] - m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2]
		+ m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1] + m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3] + m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2]
		+ m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3] + m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1] + m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2]
		- m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1] - m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3] - m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2]
		- m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0] - m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0] - m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0]
		+ m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0] + m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0] + m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0];
	resultInverse.m[0][0] = (
		m.m[1][1] * m.m[2][2] * m.m[3][3] + m.m[1][2] * m.m[2][3] * m.m[3][1] + m.m[1][3] * m.m[2][1] * m.m[3][2]
		- m.m[1][3] * m.m[2][2] * m.m[3][1] - m.m[1][2] * m.m[2][1] * m.m[3][3] - m.m[1][1] * m.m[2][3] * m.m[3][2]) / A;
	resultInverse.m[0][1] = (
		-m.m[0][1] * m.m[2][2] * m.m[3][3] - m.m[0][2] * m.m[2][3] * m.m[3][1] - m.m[0][3] * m.m[2][1] * m.m[3][2]
		+ m.m[0][3] * m.m[2][2] * m.m[3][1] + m.m[0][2] * m.m[2][1] * m.m[3][3] + m.m[0][1] * m.m[2][3] * m.m[3][2]) / A;
	resultInverse.m[0][2] = (
		m.m[0][1] * m.m[1][2] * m.m[3][3] + m.m[0][2] * m.m[1][3] * m.m[3][1] + m.m[0][3] * m.m[1][1] * m.m[3][2]
		- m.m[0][3] * m.m[1][2] * m.m[3][1] - m.m[0][2] * m.m[1][1] * m.m[3][3] - m.m[0][1] * m.m[1][3] * m.m[3][2]) / A;
	resultInverse.m[0][3] = (
		-m.m[0][1] * m.m[1][2] * m.m[2][3] - m.m[0][2] * m.m[1][3] * m.m[2][1] - m.m[0][3] * m.m[1][1] * m.m[2][2]
		+ m.m[0][3] * m.m[1][2] * m.m[2][1] + m.m[0][2] * m.m[1][1] * m.m[2][3] + m.m[0][1] * m.m[1][3] * m.m[2][2]) / A;
	resultInverse.m[1][0] = (
		-m.m[1][0] * m.m[2][2] * m.m[3][3] - m.m[1][2] * m.m[2][3] * m.m[3][0] - m.m[1][3] * m.m[2][0] * m.m[3][2]
		+ m.m[1][3] * m.m[2][2] * m.m[3][0] + m.m[1][2] * m.m[2][0] * m.m[3][3] + m.m[1][0] * m.m[2][3] * m.m[3][2]) / A;
	resultInverse.m[1][1] = (
		m.m[0][0] * m.m[2][2] * m.m[3][3] + m.m[0][2] * m.m[2][3] * m.m[3][0] + m.m[0][3] * m.m[2][0] * m.m[3][2]
		- m.m[0][3] * m.m[2][2] * m.m[3][0] - m.m[0][2] * m.m[2][0] * m.m[3][3] - m.m[0][0] * m.m[2][3] * m.m[3][2]) / A;
	resultInverse.m[1][2] = (
		-m.m[0][0] * m.m[1][2] * m.m[3][3] - m.m[0][2] * m.m[1][3] * m.m[3][0] - m.m[0][3] * m.m[1][0] * m.m[3][2]
		+ m.m[0][3] * m.m[1][2] * m.m[3][0] + m.m[0][2] * m.m[1][0] * m.m[3][3] + m.m[0][0] * m.m[1][3] * m.m[3][2]) / A;
	resultInverse.m[1][3] = (
		m.m[0][0] * m.m[1][2] * m.m[2][3] + m.m[0][2] * m.m[1][3] * m.m[2][0] + m.m[0][3] * m.m[1][0] * m.m[2][2]
		- m.m[0][3] * m.m[1][2] * m.m[2][0] - m.m[0][2] * m.m[1][0] * m.m[2][3] - m.m[0][0] * m.m[1][3] * m.m[2][2]) / A;
	resultInverse.m[2][0] = (
		m.m[1][0] * m.m[2][1] * m.m[3][3] + m.m[1][1] * m.m[2][3] * m.m[3][0] + m.m[1][3] * m.m[2][0] * m.m[3][1]
		- m.m[1][3] * m.m[2][1] * m.m[3][0] - m.m[1][1] * m.m[2][0] * m.m[3][3] - m.m[1][0] * m.m[2][3] * m.m[3][1]) / A;
	resultInverse.m[2][1] = (
		-m.m[0][0] * m.m[2][1] * m.m[3][3] - m.m[0][1] * m.m[2][3] * m.m[3][0] - m.m[0][3] * m.m[2][0] * m.m[3][1]
		+ m.m[0][3] * m.m[2][1] * m.m[3][0] + m.m[0][1] * m.m[2][0] * m.m[3][3] + m.m[0][0] * m.m[2][3] * m.m[3][1]) / A;
	resultInverse.m[2][2] = (
		m.m[0][0] * m.m[1][1] * m.m[3][3] + m.m[0][1] * m.m[1][3] * m.m[3][0] + m.m[0][3] * m.m[1][0] * m.m[3][1]
		- m.m[0][3] * m.m[1][1] * m.m[3][0] - m.m[0][1] * m.m[1][0] * m.m[3][3] - m.m[0][0] * m.m[1][3] * m.m[3][1]) / A;
	resultInverse.m[2][3] = (
		-m.m[0][0] * m.m[1][1] * m.m[2][3] - m.m[0][1] * m.m[1][3] * m.m[2][0] - m.m[0][3] * m.m[1][0] * m.m[2][1]
		+ m.m[0][3] * m.m[1][1] * m.m[2][0] + m.m[0][1] * m.m[1][0] * m.m[2][3] + m.m[0][0] * m.m[1][3] * m.m[2][1]) / A;
	resultInverse.m[3][0] = (
		-m.m[1][0] * m.m[2][1] * m.m[3][2] - m.m[1][1] * m.m[2][2] * m.m[3][0] - m.m[1][2] * m.m[2][0] * m.m[3][1]
		+ m.m[1][2] * m.m[2][1] * m.m[3][0] + m.m[1][1] * m.m[2][0] * m.m[3][2] + m.m[1][0] * m.m[2][2] * m.m[3][1]) / A;
	resultInverse.m[3][1] = (
		m.m[0][0] * m.m[2][1] * m.m[3][2] + m.m[0][1] * m.m[2][2] * m.m[3][0] + m.m[0][2] * m.m[2][0] * m.m[3][1]
		- m.m[0][2] * m.m[2][1] * m.m[3][0] - m.m[0][1] * m.m[2][0] * m.m[3][2] - m.m[0][0] * m.m[2][2] * m.m[3][1]) / A;
	resultInverse.m[3][2] = (
		-m.m[0][0] * m.m[1][1] * m.m[3][2] - m.m[0][1] * m.m[1][2] * m.m[3][0] - m.m[0][2] * m.m[1][0] * m.m[3][1]
		+ m.m[0][2] * m.m[1][1] * m.m[3][0] + m.m[0][1] * m.m[1][0] * m.m[3][2] + m.m[0][0] * m.m[1][2] * m.m[3][1]) / A;
	resultInverse.m[3][3] = (
		m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[0][1] * m.m[1][2] * m.m[2][0] + m.m[0][2] * m.m[1][0] * m.m[2][1]
		- m.m[0][2] * m.m[1][1] * m.m[2][0] - m.m[0][1] * m.m[1][0] * m.m[2][2] - m.m[0][0] * m.m[1][2] * m.m[2][1]) / A;
	return resultInverse;
}

Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 resultTransform = {};
	resultTransform.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + 1.0f * matrix.m[3][0];
	resultTransform.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + 1.0f * matrix.m[3][1];
	resultTransform.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + 1.0f * matrix.m[3][2];

	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + 1.0f * matrix.m[3][3];
	assert(w != 0.0f);
	resultTransform.x /= w;
	resultTransform.y /= w;
	resultTransform.z /= w;

	return resultTransform;
}

// アフィン変換
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3 translate) {
	Matrix4x4 resultAffineMatrix = {};
	Matrix4x4 resultRotateXYZMatrix = Multiply(MakeRotateXMatrix(rotate.x), Multiply(MakeRotateYMatrix(rotate.y), MakeRotateZMatrix(rotate.z)));
	resultAffineMatrix.m[0][0] = scale.x * resultRotateXYZMatrix.m[0][0];
	resultAffineMatrix.m[0][1] = scale.x * resultRotateXYZMatrix.m[0][1];
	resultAffineMatrix.m[0][2] = scale.x * resultRotateXYZMatrix.m[0][2];
	resultAffineMatrix.m[1][0] = scale.y * resultRotateXYZMatrix.m[1][0];
	resultAffineMatrix.m[1][1] = scale.y * resultRotateXYZMatrix.m[1][1];
	resultAffineMatrix.m[1][2] = scale.y * resultRotateXYZMatrix.m[1][2];
	resultAffineMatrix.m[2][0] = scale.z * resultRotateXYZMatrix.m[2][0];
	resultAffineMatrix.m[2][1] = scale.z * resultRotateXYZMatrix.m[2][1];
	resultAffineMatrix.m[2][2] = scale.z * resultRotateXYZMatrix.m[2][2];
	resultAffineMatrix.m[3][0] = translate.x;
	resultAffineMatrix.m[3][1] = translate.y;
	resultAffineMatrix.m[3][2] = translate.z;
	resultAffineMatrix.m[3][3] = 1;
	return resultAffineMatrix;
}

// x軸回転行列
Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 resultRotateXMatrix = {};
	resultRotateXMatrix.m[0][0] = 1;
	resultRotateXMatrix.m[1][1] = std::cos(radian);
	resultRotateXMatrix.m[1][2] = std::sin(radian);
	resultRotateXMatrix.m[2][1] = -std::sin(radian);
	resultRotateXMatrix.m[2][2] = std::cos(radian);
	resultRotateXMatrix.m[3][3] = 1;
	return resultRotateXMatrix;
}

// y軸回転行列
Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 resultRotateYMatrix = {};
	resultRotateYMatrix.m[0][0] = std::cos(radian);
	resultRotateYMatrix.m[0][2] = -std::sin(radian);
	resultRotateYMatrix.m[1][1] = 1;
	resultRotateYMatrix.m[2][0] = std::sin(radian);
	resultRotateYMatrix.m[2][2] = std::cos(radian);
	resultRotateYMatrix.m[3][3] = 1;
	return resultRotateYMatrix;
}

// z軸回転行列
Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 resultRotateZMatrix = {};
	resultRotateZMatrix.m[0][0] = std::cos(radian);
	resultRotateZMatrix.m[0][1] = std::sin(radian);
	resultRotateZMatrix.m[1][0] = -std::sin(radian);
	resultRotateZMatrix.m[1][1] = std::cos(radian);
	resultRotateZMatrix.m[2][2] = 1;
	resultRotateZMatrix.m[3][3] = 1;
	return resultRotateZMatrix;
}

// 透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 resultPerspectiveFovMatrix = {};
	resultPerspectiveFovMatrix.m[0][0] = (1 / aspectRatio) * (1 / std::tan(fovY / 2));
	resultPerspectiveFovMatrix.m[1][1] = 1 / std::tan(fovY / 2);
	resultPerspectiveFovMatrix.m[2][2] = farClip / (farClip - nearClip);
	resultPerspectiveFovMatrix.m[2][3] = 1;
	resultPerspectiveFovMatrix.m[3][2] = -nearClip * farClip / (farClip - nearClip);
	return resultPerspectiveFovMatrix;
}

// ビューポート変換
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 resultViewPortMatrix = {};
	resultViewPortMatrix.m[0][0] = width / 2.0f;
	resultViewPortMatrix.m[1][1] = -height / 2.0f;
	resultViewPortMatrix.m[2][2] = maxDepth - minDepth;
	resultViewPortMatrix.m[3][0] = left + (width / 2);
	resultViewPortMatrix.m[3][1] = top + (height / 2);
	resultViewPortMatrix.m[3][2] = minDepth;
	resultViewPortMatrix.m[3][3] = 1;
	return resultViewPortMatrix;
}

Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 resultCross = {};
	resultCross.x = v1.y * v2.z - v1.z * v2.y;
	resultCross.y = v1.z * v2.x - v1.x * v2.z;
	resultCross.z = v1.x * v2.y - v1.y * v2.x;
	return resultCross;
}

// 加算
Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 AddResult = {};
	AddResult.x = v1.x + v2.x;
	AddResult.y = v1.y + v2.y;
	AddResult.z = v1.z + v2.z;
	return AddResult;
}
// 減算
Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 SubtractResult = {};
	SubtractResult.x = v1.x - v2.x;
	SubtractResult.y = v1.y - v2.y;
	SubtractResult.z = v1.z - v2.z;
	return SubtractResult;
}

Vector3 Multiply(const Vector3& v1, const Vector3& v2) {
	return { v1.x * v2.x,v1.y * v2.y,v1.z * v2.z };
}

// 行列の加法
Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 resultAdd = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			resultAdd.m[j][i] = m1.m[j][i] + m2.m[j][i];
		}
	}
	return resultAdd;
}
// 行列の減法
Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 resultSubtract = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			resultSubtract.m[j][i] = m1.m[j][i] - m2.m[j][i];
		}
	}
	return resultSubtract;
}

// スカラー倍
Vector3 Multiply(float scalar, const Vector3& v) {
	Vector3 MultiplyResult = {};
	MultiplyResult.x = scalar * v.x;
	MultiplyResult.y = scalar * v.y;
	MultiplyResult.z = scalar * v.z;
	return MultiplyResult;
}

// 内積
float Dot(const Vector3& v1, const Vector3& v2) {
	float DotResult = {};
	DotResult = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	return DotResult;
}
// 長さ
float Length(const Vector3& v) {
	float LengthResult = {};
	LengthResult = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	return LengthResult;
}

// 正規化
Vector3 Normalize(const Vector3 v) {
	Vector3 NormalizeResult = {};
	float LengthResult = {};
	LengthResult = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	NormalizeResult.x = v.x / LengthResult;
	NormalizeResult.y = v.y / LengthResult;
	NormalizeResult.z = v.z / LengthResult;
	return NormalizeResult;
}

Vector3 Project(const Vector3& v1, const Vector3& v2) {
	return Multiply(Dot(v1, v2) / powf(Length(v2), 2), v2);
}

Vector3 ClosestPoint(const Vector3& point, const Segment& segment) {
	Vector3 segmentVector = segment.diff;

	Vector3 pointToOrigin = Subtract(point, segment.origin);

	float t = Dot(pointToOrigin, segmentVector) / Dot(segmentVector, segmentVector);

	Vector3 closestPointOnSegment = Add(segment.origin, Multiply(t, segmentVector));

	return closestPointOnSegment;
}

// 線分の描画
void DrawSegment(const Segment& segment, const Matrix4x4& worldViewProjectionMatrix, const Matrix4x4& viewPortMatrix) {
	Vector3 start = Transform(Transform(segment.origin, worldViewProjectionMatrix), viewPortMatrix);
	Vector3 end = Transform(Transform(Add(segment.origin, segment.diff), worldViewProjectionMatrix), viewPortMatrix);
	Novice::DrawLine(static_cast<int>(start.x), static_cast<int>(start.y), static_cast<int>(end.x), static_cast<int>(end.y), WHITE);
}

bool IsCollision(const Sphere& s1, const Sphere& s2) {
	// 2つの弾の中心点間の距離を求める
	float distance = Length(s2.center - s1.center);
	// 半径の合計よりも短ければ衝突
	if (distance <= s1.radius + s2.radius) {
		return true;
	} else {
		return false;
	}
}

bool IsCollision(const Sphere& s, const Plane& p) {
	// 平面との距離を求める
	float k = Dot(p.normal, s.center) - p.distance / Length(p.normal);
	// 絶対値をとる
	if (k < 0) {
		k *= -1;
	}
	if (k <= s.radius) {
		return true;
	} else {
		return false;
	}
}

bool IsCollision(const Plane& p, const Segment& s) {
	// 法線と線の内積を求める
	float dot = Dot(p.normal, s.diff);

	// 平行の場合衝突していない
	if (dot == 0.0f) {
		return false;
	}
	// tを求める
	float t = (p.distance - Dot(s.origin, p.normal)) / dot;

	if (t == -1 || t == 2) {
		return false;
	} else {
		return true;
	}
}

bool IsCollision(const AABB& a, const AABB& b) {
	if ((a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z)) {
		return true;
	} else {
		return false;
	}
}

bool IsCollision(const AABB& aabb, const Sphere& sphere) {
	// 最近接点を求める
	Vector3 closestPoint{ std::clamp(sphere.center.x,aabb.min.x,aabb.max.x),
	std::clamp(sphere.center.y,aabb.min.y,aabb.max.y),std::clamp(sphere.center.z,aabb.min.z,aabb.max.z) };
	// 最近接点と球の中心との距離を求める
	float distance = Length(closestPoint - sphere.center);
	// 距離が半径よりも小さければ衝突
	if (distance <= sphere.radius) {
		return true;
	} else {
		return false;
	}
}

bool IsCollision(const AABB& aabb, const Segment& segment) {
	float dot{};
	dot = Dot(aabb.min, segment.diff);
	float tXmin = aabb.min.x - segment.origin.x / dot;
	float tXmax = aabb.max.x - segment.origin.x / dot;

	float tYmin = aabb.min.y - segment.origin.y / dot;
	float tYmax = aabb.max.y - segment.origin.y / dot;

	float tZmin = aabb.min.z - segment.origin.z / dot;
	float tZmax = aabb.max.z - segment.origin.z / dot;

	float tNearX = min(tXmin, tXmax);
	float tNearY = min(tYmin, tYmax);
	float tNearZ = min(tZmin, tZmax);
	float tFarX = max(tXmin, tXmax);
	float tFarY = max(tYmin, tYmax);
	float tFarZ = max(tZmin, tZmax);
	// AABBとの衝突点(貫通点)のtが小さい方
	float tmin = max(max(tNearX, tNearY), tNearZ);
	// AABBとの衝突点(貫通点)のtが大きい方
	float tmax = min(min(tFarX, tFarY), tFarZ);
	if (tmin <= tmax) {
		return true;
	} else {
		return false;
	}
}

Vector3 Perpendicular(const Vector3& v) {
	if (v.x != 0.0f || v.y != 0.0f) {
		return { -v.y,v.x,0.0f };
	}
	return { 0.0f,-v.z,v.y };
}

void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix) {
	Vector3 center = Multiply(plane.distance, plane.normal);
	Vector3 perpendiculars[4];
	perpendiculars[0] = Normalize(Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y,-perpendiculars[0].z };
	perpendiculars[2] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = Multiply(2.0f, perpendiculars[index]);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix), viewPortMatrix);
	}
	// pointsをそれぞれ結んでDrawLineで矩形を描画する
	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[2].x, (int)points[2].y, plane.color);
	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[3].x, (int)points[3].y, plane.color);
	Novice::DrawLine((int)points[1].x, (int)points[1].y, (int)points[2].x, (int)points[2].y, plane.color);
	Novice::DrawLine((int)points[1].x, (int)points[1].y, (int)points[3].x, (int)points[3].y, plane.color);
}

void Hook(Spring& s, Ball& b) {
	float deltaTime = 1.0f / 60.0f;
	Vector3 diff = b.position - s.anchor;
	float length = Length(diff);
	if (length != 0.0f) {
		Vector3 direction = Normalize(diff);
		Vector3 restPosition = s.anchor + direction * s.naturalLength;
		Vector3 displacement = length * (b.position - restPosition);
		Vector3 restoringForce = -s.stiffness * displacement;
		// 減衰抵抗を計算する
		Vector3 dampingForce = -s.dampingCoefficient * b.velocity;
		// 減衰抵抗も加算して、物体にかかる力を決定する
		Vector3 force = restoringForce + dampingForce;
		b.acceleration = force / b.mass;
	}
	// 加速度も速度どちらも秒を基準とした値である
	// それが、1/60秒間(deltaTime)適用されたと考える
	b.velocity = b.velocity + b.acceleration * deltaTime;
	b.position = b.position + b.velocity * deltaTime;
}

void circularMotion(Ball& b, Vector3 c) {
	float deltaTime = 1.0f / 60.0f;
	b.angle += b.angularVelocity * deltaTime;
	b.position.x = c.x + std::cos(b.angle) * b.radius;
	b.position.y = c.y + std::sin(b.angle) * b.radius;
	b.position.z = c.z;
	b.velocity = {
	-b.radius * b.angularVelocity * std::sin(b.angle),
	b.radius * b.angularVelocity * std::cos(b.angle),0 };
	b.acceleration =
	{ -b.radius * (b.angularVelocity * b.angularVelocity) * std::cos(b.angle),
		-b.radius * (b.angularVelocity * b.angularVelocity) * std::sin(b.angle),0 };

	b.velocity = b.velocity + b.acceleration * deltaTime;
	b.position = b.position + b.velocity * deltaTime;
}

void DrawEllipse(Ball b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix) {
	b.position = Transform(b.position, Multiply(viewProjectionMatrix, viewPortMatrix));
	Novice::DrawEllipse(int(b.position.x), int(b.position.y), int(10), int(10), 0.0f, b.color, kFillModeSolid);
}

void PendulumSwing(Pendulum& p, Ball& b) {
	float deltaTime = 1.0f / 60.0f;
	p.anglarAcceleration = (-9.8f / p.length) * std::sin(p.angle);
	p.anglarVelocity = p.anglarVelocity + p.anglarAcceleration * deltaTime;
	p.angle = p.angle + p.anglarVelocity * deltaTime;

	b.position.x = p.anchor.x + std::sin(p.angle) * p.length;
	b.position.y = p.anchor.y - std::cos(p.angle) * p.length;
	b.position.z = p.anchor.z;
}

void DrawPendulum(Pendulum p, Ball b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix) {
	b.position = Transform(b.position, Multiply(viewProjectionMatrix, viewPortMatrix));
	p.anchor = Transform(p.anchor, Multiply(viewProjectionMatrix, viewPortMatrix));
	Novice::DrawEllipse(int(b.position.x), int(b.position.y), int(b.radius), int(b.radius), 0.0f, b.color, kFillModeSolid);
	Novice::DrawLine(int(p.anchor.x), int(p.anchor.y), int(b.position.x), int(b.position.y), 0xFFFFFFFF);
}

void ConicalPendulumSwing(ConicalPendulum& p, Ball& b) {
	float deltaTime = 1.0f / 60.0f;
	p.anglarVelocity = std::sqrt(9.8f / (p.length * std::cos(p.halfApexAngle)));
	p.angle = p.angle + p.anglarVelocity * deltaTime;

	float radius = std::sin(p.halfApexAngle) * p.length;
	float height = std::cos(p.halfApexAngle) * p.length;
	b.position.x = p.anchor.x + std::cos(p.angle) * radius;
	b.position.y = p.anchor.y - height;
	b.position.z = p.anchor.z - std::sin(p.angle) * radius;
}

void DrawConicalPendulum(ConicalPendulum p, Ball b, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix) {
	b.position = Transform(b.position, Multiply(viewProjectionMatrix, viewPortMatrix));
	p.anchor = Transform(p.anchor, Multiply(viewProjectionMatrix, viewPortMatrix));
	Novice::DrawEllipse(int(b.position.x), int(b.position.y), int(b.radius), int(b.radius), 0.0f, b.color, kFillModeSolid);
	Novice::DrawLine(int(p.anchor.x), int(p.anchor.y), int(b.position.x), int(b.position.y), 0xFFFFFFFF);
}

Vector3 Reflect(const Vector3& input, const Vector3& normal){
	Vector3 r = input - 2 * Dot(input,normal) * normal;
	return r;
}

void BallReflect(Ball& b,Plane p){
	float deltaTime = 1.0f / 60.0f;
	b.velocity = b.velocity + b.acceleration * deltaTime;
	b.position = b.position + b.velocity * deltaTime;
	if (IsCollision(Sphere{ b.position,b.radius }, p)) {
		Vector3 reflected = Reflect(b.velocity, p.normal);
		Vector3 projectToNormal = Project(reflected, p.normal);
		Vector3 movingDirection = reflected - projectToNormal;
		b.velocity = projectToNormal * 0.5f + movingDirection;
	}
}

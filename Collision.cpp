#include "Collision.h"
#include"calc.h"
#include"MyVector3.h"
#include<cmath>
#include<algorithm>

bool Collision::IsCollision(const Sphere& s1, const Sphere& s2) {
	float length = Calc::MakeLength(s1.center_, s2.center_);

	//スケールはいじるな！お願いします。
	if (length <= s1.radius_ + s2.radius_) {
		return true;
	}
	return false;
}

bool Collision::IsCollision(const Sphere& sphere, const Plane& plane) {
	float length = Calc::MakeLength(sphere, plane);

	if (length <= sphere.radius_) {
		if (plane.isLimit) {
			MyVector3 center = plane.normal;
			center *= plane.distance;

			MyVector3 perpendiculars[4];

			perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
			perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
			perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
			perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

			for (int32_t index = 0; index < 4; index++) {
				perpendiculars[index] = center + perpendiculars[index] * 2.0f;
			}

			MyVector3 pos = Calc::Project(center - sphere.center_, plane.normal) + sphere.center_;
			float radius = sqrtf(powf(sphere.radius_, 2.0f) - powf(length, 2.0f));

			float len;
			len = Calc::MakeLength(pos, perpendiculars[0], perpendiculars[2]);
			if (len <= radius) {
				return true;
			}
			len = Calc::MakeLength(pos, perpendiculars[0], perpendiculars[3]);
			if (len <= radius) {
				return true;
			}
			len = Calc::MakeLength(pos, perpendiculars[1], perpendiculars[2]);
			if (len <= radius) {
				return true;
			}
			len = Calc::MakeLength(pos, perpendiculars[1], perpendiculars[2]);
			if (len <= radius) {
				return true;
			}

			float dot[4] = {};

			dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
			dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
			dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
			dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

			if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
				return true;
			}
		}
		else {
			return true;
		}
	}
	return false;
}

bool Collision::IsCollision(const Segment& segment, const Plane& plane) {

	if (Calc::Dot(segment.diff, plane.normal) == 0) {

		MyVector3 pos = plane.normal;
		pos *= plane.distance;

		Line tmpLine = { segment.origin,plane.normal };

		pos = Calc::ClosestPoint(pos, tmpLine);

		if (Calc::MakeLength(pos, segment.origin) == 0) {

			return true;
		}
		return false;
	}

	float t = (plane.distance - Calc::Dot(segment.origin, plane.normal)) / Calc::Dot(segment.diff, plane.normal);

	if (t >= 0 && t <= 1) {
		if (plane.isLimit) {
			return IsInPlane(segment, plane);
		}

		return true;
	}
	return false;

}
bool Collision::IsCollision(const Ray& ray, const Plane& plane) {
	if (Calc::Dot(ray.diff, plane.normal) == 0) {
		MyVector3 pos = plane.normal;
		pos *= plane.distance;

		Line tmpLine = { ray.origin,plane.normal };

		pos = Calc::ClosestPoint(pos, tmpLine);

		if (Calc::MakeLength(pos, ray.origin) == 0) {

			return true;
		}
		return false;
	}

	float t = (plane.distance - Calc::Dot(ray.origin, plane.normal)) / Calc::Dot(ray.diff, plane.normal);

	if (t >= 0) {
		if (plane.isLimit) {
			return IsInPlane(ray, plane);
		}

		return true;
	}
	return false;

}
bool Collision::IsCollision(const Line& line, const Plane& plane) {

	if (Calc::Dot(line.diff, plane.normal) == 0) {
		MyVector3 pos = plane.normal;
		pos *= plane.distance;

		Line tmpLine = { line.origin,plane.normal };

		pos = Calc::ClosestPoint(pos, tmpLine);

		if (Calc::MakeLength(pos, line.origin) == 0) {

			return true;
		}
		return false;
	}

	if (plane.isLimit) {
		return IsInPlane(line, plane);
	}

	return true;
}

bool Collision::IsInPlane(const Segment& segment, const Plane& plane) {

	MyVector3 pos = plane.normal;
	pos *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	for (int32_t index = 0; index < 4; index++) {
		perpendiculars[index] = pos + perpendiculars[index] * 2.0f;
	}

	float t = (plane.distance - Calc::Dot(segment.origin, plane.normal)) / Calc::Dot(segment.diff, plane.normal);

	pos = MyVector3(segment.origin) + MyVector3(segment.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
	dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
	dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
	dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsInPlane(const Ray& ray, const Plane& plane) {

	MyVector3 pos = plane.normal;
	pos *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	for (int32_t index = 0; index < 4; index++) {
		perpendiculars[index] = pos + perpendiculars[index] * 2.0f;
	}

	float t = (plane.distance - Calc::Dot(ray.origin, plane.normal)) / Calc::Dot(ray.diff, plane.normal);

	pos = MyVector3(ray.origin) + MyVector3(ray.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
	dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
	dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
	dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsInPlane(const Line& line, const Plane& plane) {

	MyVector3 pos = plane.normal;
	pos *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	for (int32_t index = 0; index < 4; index++) {
		perpendiculars[index] = pos + perpendiculars[index] * 2.0f;
	}

	float t = (plane.distance - Calc::Dot(line.origin, plane.normal)) / Calc::Dot(line.diff, plane.normal);

	pos = MyVector3(line.origin) + MyVector3(line.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
	dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
	dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
	dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const Triangle& triangle, const Segment& segment) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(triangle.vertices[1]) - MyVector3(triangle.vertices[0]),
		MyVector3(triangle.vertices[2]) - MyVector3(triangle.vertices[1])));

	if (Calc::Dot(segment.diff, normal) == 0) {

		/*Line tmpLine = { segment.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, segment.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal};

	float distance = Calc::MakeLength(Calc::ClosestPoint(triangle.vertices[0], tmpLine));

	float t = (distance - Calc::Dot(segment.origin, normal)) / Calc::Dot(segment.diff, normal);

	if (t >= 0 && t <= 1) {
		MyVector3 pos = MyVector3(segment.origin) + MyVector3(segment.diff) * t;

		float dot[3] = {};

		dot[0] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[2], MyVector3(triangle.vertices[0]) - pos));
		dot[1] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[1], MyVector3(triangle.vertices[2]) - pos));
		dot[2] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[0], MyVector3(triangle.vertices[1]) - pos));

		if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0)) {
			return true;
		}
	}

	return false;
}

bool Collision::IsCollision(const Triangle& triangle, const Ray& ray) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(triangle.vertices[1]) - MyVector3(triangle.vertices[0]),
		MyVector3(triangle.vertices[2]) - MyVector3(triangle.vertices[1])));

	if (Calc::Dot(ray.diff, normal) == 0) {

		/*Line tmpLine = { ray.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, ray.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal };

	float distance = Calc::MakeLength(Calc::ClosestPoint(triangle.vertices[0], tmpLine));

	float t = (distance - Calc::Dot(ray.origin, normal)) / Calc::Dot(ray.diff, normal);

	if (t >= 0) {
		MyVector3 pos = MyVector3(ray.origin) + MyVector3(ray.diff) * t;

		float dot[3] = {};

		dot[0] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[2], MyVector3(triangle.vertices[0]) - pos));
		dot[1] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[1], MyVector3(triangle.vertices[2]) - pos));
		dot[2] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[0], MyVector3(triangle.vertices[1]) - pos));

		if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0)) {
			return true;
		}
	}

	return false;
}

bool Collision::IsCollision(const Triangle& triangle, const Line& line) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(triangle.vertices[1]) - MyVector3(triangle.vertices[0]),
		MyVector3(triangle.vertices[2]) - MyVector3(triangle.vertices[1])));

	if (Calc::Dot(line.diff, normal) == 0) {

		/*Line tmpLine = { line.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, line.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal };

	float distance = Calc::MakeLength(Calc::ClosestPoint(triangle.vertices[0], tmpLine));

	float t = (distance - Calc::Dot(line.origin, normal)) / Calc::Dot(line.diff, normal);

	MyVector3 pos = MyVector3(line.origin) + MyVector3(line.diff) * t;

	float dot[3] = {};

	dot[0] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[2], MyVector3(triangle.vertices[0]) - pos));
	dot[1] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[1], MyVector3(triangle.vertices[2]) - pos));
	dot[2] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[0], MyVector3(triangle.vertices[1]) - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const AABB& a, const AABB& b) {

	if ((a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z)) {
		return true;
	}
	return false;
}

bool Collision::IsCollision(const AABB& a, const Sphere& sphere) {
	MyVector3 closestPoint = { std::clamp(sphere.center_.x,a.min.x,a.max.x),
		std::clamp(sphere.center_.y,a.min.y,a.max.y),
		std::clamp(sphere.center_.z,a.min.z,a.max.z) };

	float distance = Calc::MakeLength(closestPoint, sphere.center_);

	if (distance <= sphere.radius_) {
		return true;
	}

	return false;
}
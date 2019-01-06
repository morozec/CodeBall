#include "DanCalculator.h"
#include "Helper.h"

const double DanCalculator::Eps = 1E-6;

Vector3D DanCalculator::_vector000;
Vector3D DanCalculator::_vector010;
Vector3D DanCalculator::_vector0M10;
Vector3D DanCalculator::_vector0Ah0;
Vector3D DanCalculator::_vectorAw200;
Vector3D DanCalculator::_vector00Ad2Plus;
Vector3D DanCalculator::_vector00M1;
Vector3D DanCalculator::_vector00Ad2;
Vector3D DanCalculator::_vectorAgw200;
Vector3D DanCalculator::_vectorM100;
Vector3D DanCalculator::_vector0Agh0;

Vector DanCalculator::_topCornerO;
Vector DanCalculator::_goalOuterCorner;
Vector DanCalculator::_cornerO;

void DanCalculator::Init(const model::Arena & arena)
{
	_vector000 = Vector3D(0, 0, 0);
	_vector010 = Vector3D(0, 1, 0);
	_vector0M10 = Vector3D(0, -1, 0);
	_vector0Ah0 = Vector3D(0, arena.height, 0);
	_vectorAw200 = Vector3D(arena.width / 2, 0, 0);
	_vector00Ad2Plus = Vector3D(0, 0, (arena.depth / 2) + arena.goal_depth);
	_vector00M1 = Vector3D(0, 0, -1);
	_vector00Ad2 = Vector3D(0, 0, arena.depth / 2);

	_vectorAgw200 = Vector3D(arena.goal_width / 2, 0, 0);
	_vectorM100 = Vector3D(-1, 0, 0);
	_vector0Agh0 = Vector3D(0, arena.goal_height, 0);

	_topCornerO = Vector(
		(arena.goal_width / 2) - arena.goal_top_radius,
		arena.goal_height - arena.goal_top_radius
	);
	_goalOuterCorner = Vector(
		(arena.goal_width / 2) + arena.goal_side_radius,
		(arena.depth / 2) + arena.goal_side_radius
	);

	_cornerO = Vector(
		(arena.width / 2) - arena.corner_radius,
		(arena.depth / 2) - arena.corner_radius
	);

}

Dan DanCalculator::Min(Dan & dan1, Dan dan2)
{
	return dan1.Distance <= dan2.Distance ? dan1 : dan2;
}

Dan DanCalculator::GetDanToPlane(const Vector3D& point, const Vector3D& pointOnPlane, const Vector3D& planeNormal)
{
	return Dan(
		(point.X - pointOnPlane.X) * planeNormal.X +
		(point.Y - pointOnPlane.Y) * planeNormal.Y +
		(point.Z - pointOnPlane.Z) * planeNormal.Z,
		planeNormal);
}

Dan DanCalculator::GetDanToSphereInner(const Vector3D& point, const Vector3D& sphereCenter, double sphereRadius)
{
	return Dan(
		sphereRadius - Helper::GetLength(point, sphereCenter),
		Helper::GetTargetVelocity(point, sphereCenter, 1));
}

Dan DanCalculator::GetDanToSphereOuter(const Vector3D& point, const Vector3D& sphereCenter, double sphereRadius)
{
	double dx = point.X - sphereCenter.X;
	double dy = point.Y - sphereCenter.Y;
	double dz = point.Z - sphereCenter.Z;
	double length = sqrt(dx * dx + dy * dy + dz * dz);
	Vector3D normal = Vector3D(dx / length, dy / length, dz / length);
	return Dan(
		length - sphereRadius,
		normal);
}

Dan DanCalculator::GetDanToArenaQuarter(const Vector3D & point, const model::Arena & arena)
{
	// Ground
	Dan dan = GetDanToPlane(point, _vector000, _vector010);
	// Ceiling
	dan = Min(dan,GetDanToPlane(point, _vector0Ah0, _vector0M10));
	// Side x
	dan = Min(dan, GetDanToPlane(point, _vectorAw200, _vectorM100));
	// Side z (goal)
	dan = Min(dan,
		GetDanToPlane(
			point,
			_vector00Ad2Plus,
			_vector00M1));
	// Side z
	Vector v = Vector(point.X - ((arena.goal_width / 2) - arena.goal_top_radius),
		point.Y - (arena.goal_height - arena.goal_top_radius));


	if (point.X >= (arena.goal_width / 2) + arena.goal_side_radius
		|| point.Y >= arena.goal_height + arena.goal_side_radius
		|| (
			v.X > 0
			&& v.Y > 0
			&& v.Length() >= arena.goal_top_radius + arena.goal_side_radius))
	{
		dan = Min(dan, GetDanToPlane(point, _vector00Ad2, _vector00M1));
	}


	// Side x & ceiling (goal)
	if (point.Z >= (arena.depth / 2) + arena.goal_side_radius)
	{
		// x
		dan = Min(dan,
			GetDanToPlane(
				point,
				_vectorAgw200,
				_vectorM100));
		// y
		dan = Min(dan, GetDanToPlane(point, _vector0Agh0, _vector0M10));
	}

	// Goal back corners
	if (abs(arena.bottom_radius - arena.goal_top_radius) > Eps) throw "Not same radiuses";
	if (point.Z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius)
	{
		dan = Min(dan,
			GetDanToSphereInner(
				point,
				Vector3D(
					Helper::Clamp(
						point.X,
						arena.bottom_radius - (arena.goal_width / 2),
						(arena.goal_width / 2) - arena.bottom_radius
					),
					Helper::Clamp(
						point.Y,
						arena.bottom_radius,
						arena.goal_height - arena.goal_top_radius
					),
					(arena.depth / 2) + arena.goal_depth - arena.bottom_radius),
				arena.bottom_radius));
	}

	// Corner
	if (point.X > (arena.width / 2) - arena.corner_radius
		&& point.Z > (arena.depth / 2) - arena.corner_radius)
	{
		dan = Min(dan,
			GetDanToSphereInner(
				point,
				Vector3D(
				(arena.width / 2) - arena.corner_radius,
					point.Y,
					(arena.depth / 2) - arena.corner_radius
				),
				arena.corner_radius));
	}

	// Goal outer corner
	if (point.Z < (arena.depth / 2) + arena.goal_side_radius)
	{
		// Side x
		if (point.X < (arena.goal_width / 2) + arena.goal_side_radius)
		{
			dan = Min(dan,
				GetDanToSphereOuter(
					point,
					Vector3D(
					(arena.goal_width / 2) + arena.goal_side_radius,
						point.Y,
						(arena.depth / 2) + arena.goal_side_radius
					),
					arena.goal_side_radius));
		}

		// Ceiling
		if (point.Y < arena.goal_height + arena.goal_side_radius)
		{
			dan = Min(dan,
				GetDanToSphereOuter(
					point,
					Vector3D(
						point.X,
						arena.goal_height + arena.goal_side_radius,
						(arena.depth / 2) + arena.goal_side_radius
					),
					arena.goal_side_radius));
		}

		// Top corner
		v = Vector(point.X - _topCornerO.X, point.Y - _topCornerO.Y);
		if (v.X > 0 && v.Y > 0)
		{
			Vector o = _topCornerO + v.Normalize().Mult(arena.goal_top_radius + arena.goal_side_radius);
			dan = Min(dan,
				GetDanToSphereOuter(
					point,
					Vector3D(o.X, o.Y, (arena.depth / 2) + arena.goal_side_radius),
					arena.goal_side_radius));
		}
	}


	// Goal inside top corners
	if (point.Z > (arena.depth / 2) + arena.goal_side_radius
		&& point.Y > arena.goal_height - arena.goal_top_radius)
	{
		// Side x
		if (point.X > (arena.goal_width / 2) - arena.goal_top_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
					(arena.goal_width / 2) - arena.goal_top_radius,
						arena.goal_height - arena.goal_top_radius,
						point.Z
					),
					arena.goal_top_radius));
		}

		// Side z
		if (point.Z > (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
						point.X,
						arena.goal_height - arena.goal_top_radius,
						(arena.depth / 2) + arena.goal_depth - arena.goal_top_radius
					),
					arena.goal_top_radius));
		}
	}


	// Bottom corners
	if (point.Y < arena.bottom_radius)
	{
		// Side x
		if (point.X > (arena.width / 2) - arena.bottom_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
					(arena.width / 2) - arena.bottom_radius,
						arena.bottom_radius,
						point.Z
					),
					arena.bottom_radius));
		}

		// Side z
		if (point.Z > (arena.depth / 2) - arena.bottom_radius
			&& point.X >= (arena.goal_width / 2) + arena.goal_side_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
						point.X,
						arena.bottom_radius,
						(arena.depth / 2) - arena.bottom_radius
					),
					arena.bottom_radius));
		}

		// Side z (goal)
		if (point.Z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
						point.X,
						arena.bottom_radius,
						(arena.depth / 2) + arena.goal_depth - arena.bottom_radius
					),
					arena.bottom_radius));
		}

		// Goal outer corner
		v = Vector(point.X - _goalOuterCorner.X, point.Z - _goalOuterCorner.Y);
		if (v.X < 0 && v.Y < 0
			&& v.Length() < arena.goal_side_radius + arena.bottom_radius)
		{
			Vector o = _goalOuterCorner + v.Normalize().Mult(arena.goal_side_radius + arena.bottom_radius);
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(o.X, arena.bottom_radius, o.Y),
					arena.bottom_radius));
		}

		// Side x (goal)
		if (point.Z >= (arena.depth / 2) + arena.goal_side_radius
			&& point.X > (arena.goal_width / 2) - arena.bottom_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
					(arena.goal_width / 2) - arena.bottom_radius,
						arena.bottom_radius,
						point.Z
					),
					arena.bottom_radius));
		}

		// Corner
		if (point.X > (arena.width / 2) - arena.corner_radius
			&& point.Z > (arena.depth / 2) - arena.corner_radius)
		{
			Vector n = Vector(point.X - _cornerO.X, point.Z - _cornerO.Y);
			double dist = n.Length();
			if (dist > arena.corner_radius - arena.bottom_radius)
			{
				n = n.Mult(1 / dist);
				Vector o2 = _cornerO + n.Mult(arena.corner_radius - arena.bottom_radius);
				dan = Min(dan,
					GetDanToSphereInner(
						point,
						Vector3D(o2.X, arena.bottom_radius, o2.Y),
						arena.bottom_radius));
			}
		}
	}

	// Ceiling corners
	if (point.Y > arena.height - arena.top_radius)
	{
		// Side x
		if (point.X > (arena.width / 2) - arena.top_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
					(arena.width / 2) - arena.top_radius,
						arena.height - arena.top_radius,
						point.Z
					),
					arena.top_radius));
		}

		// Side z
		if (point.Z > (arena.depth / 2) - arena.top_radius)
		{
			dan = Min(dan,
				GetDanToSphereInner(
					point,
					Vector3D(
						point.X,
						arena.height - arena.top_radius,
						(arena.depth / 2) - arena.top_radius
					),
					arena.top_radius));
		}

		// Corner
		if (point.X > (arena.width / 2) - arena.corner_radius
			&& point.Z > (arena.depth / 2) - arena.corner_radius)
		{

			Vector dv = Vector(point.X - _cornerO.X, point.Z - _cornerO.Y);
			if (dv.Length() > arena.corner_radius - arena.top_radius)
			{
				Vector n = dv.Normalize();
				Vector o2 = _cornerO + n.Mult(arena.corner_radius - arena.top_radius);
				dan = Min(dan,
					GetDanToSphereInner(
						point,
						Vector3D(o2.X, arena.height - arena.top_radius, o2.Y),
						arena.top_radius));
			}
		}
	}

	return Dan(dan.Distance, Vector3D(dan.Normal)); //чтобы не перетереть _vector000 и пр.
}

Dan DanCalculator::GetDanToArena(Vector3D point, const model::Arena & arena)
{
	//TODO pass point by reference
	bool negateX = point.X < 0;
	bool negateZ = point.Z < 0;
	if (negateX) point.X *= -1;
	if (negateZ) point.Z *= -1;
	Dan result = GetDanToArenaQuarter(point, arena);
	if (negateX) result.Normal.X = -result.Normal.X;
	if (negateZ) result.Normal.Z = -result.Normal.Z;
	return result;
}

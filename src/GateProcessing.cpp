#include "GateProcessing.h"
#include <Python.h>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

GateProcessing::GateProcessing(double scalar = 1.0, int number_of_gates = 4)
{

	/* UP LEFT */						//X       //Y
	corner_points.push_back(cv::Point3f(-0.4329, -0.2955, -0.1 / scalar) * scalar);   // 1 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, -0.3301, -0.1 / scalar) * scalar);   // 2 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, -0.3647, -0.1 / scalar) * scalar);   // 3 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, -0.3983, -0.1 / scalar) * scalar);   // 4 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, -0.4329, -0.1 / scalar) * scalar);   // 5 baixo,cima
	corner_points.push_back(cv::Point3f(-0.3983, -0.4329, -0.1 / scalar) * scalar);   // 6 esquerda,direita
	corner_points.push_back(cv::Point3f(-0.3647, -0.4329, -0.1 / scalar) * scalar);   // 7 esquerda,direita
	corner_points.push_back(cv::Point3f(-0.3301, -0.4329, -0.1 / scalar) * scalar);   // 8 esquerda,direita
	corner_points.push_back(cv::Point3f(-0.2955, -0.4329, -0.1 / scalar) * scalar);   // 9 esquerda,direita
	corner_points.push_back(cv::Point3f(-0.2619, -0.4329, -0.1 / scalar) * scalar);   // 10 esquerda,direita
	corners_points.push_back(corner_points);

	/* UP RIGHT */
	corner_points.clear();				//X       //Y
	corner_points.push_back(cv::Point3f(0.2608, -0.4329, -0.1 / scalar) * scalar);   // 1 esquerda,direita
	corner_points.push_back(cv::Point3f(0.2944, -0.4329, -0.1 / scalar) * scalar);   // 2 esquerda,direita
	corner_points.push_back(cv::Point3f(0.3290, -0.4329, -0.1 / scalar) * scalar);   // 3 esquerda,direita
	corner_points.push_back(cv::Point3f(0.3636, -0.4329, -0.1 / scalar) * scalar);   // 4 esquerda,direita
	corner_points.push_back(cv::Point3f(0.3972, -0.4329, -0.1 / scalar) * scalar);   // 5 esquerda,direita
	corner_points.push_back(cv::Point3f(0.4318, -0.4329, -0.1 / scalar) * scalar);   // 6 esquerda,direita
	corner_points.push_back(cv::Point3f(0.4318, -0.3983, -0.1 / scalar) * scalar);   // 7 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, -0.3647, -0.1 / scalar) * scalar);   // 8 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, -0.3301, -0.1 / scalar) * scalar);   // 9 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, -0.2955, -0.1 / scalar) * scalar);   // 10 cima,baixo
	corners_points.push_back(corner_points);

	/* DOWN RIGHT */
	corner_points.clear();				//X       //Y
	corner_points.push_back(cv::Point3f(0.4318, 0.2955, -0.1 / scalar) * scalar);   // 1 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, 0.3290, -0.1 / scalar) * scalar);   // 2 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, 0.3636, -0.1 / scalar) * scalar);   // 3 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, 0.3972, -0.1 / scalar) * scalar);   // 4 cima,baixo
	corner_points.push_back(cv::Point3f(0.4318, 0.4307, -0.1 / scalar) * scalar);   // 5 cima,baixo
	corner_points.push_back(cv::Point3f(0.3972, 0.4307, -0.1 / scalar) * scalar);	// 6 direita,esquerda
	corner_points.push_back(cv::Point3f(0.3636, 0.4307, -0.1 / scalar) * scalar);	// 7 direita,esquerda
	corner_points.push_back(cv::Point3f(0.3290, 0.4307, -0.1 / scalar) * scalar);	// 8 direita,esquerda
	corner_points.push_back(cv::Point3f(0.2944, 0.4307, -0.1 / scalar) * scalar);	// 9 direita,esquerda
	corner_points.push_back(cv::Point3f(0.2608, 0.4307, -0.1 / scalar) * scalar);	// 10 direita,esquerda
	corners_points.push_back(corner_points);

	/* DOWN LEFT */
	corner_points.clear();				//X       //Y
	corner_points.push_back(cv::Point3f(-0.2619, 0.4307, -0.1 / scalar) * scalar);	// 1 direita,esquerda
	corner_points.push_back(cv::Point3f(-0.2955, 0.4307, -0.1 / scalar) * scalar);	// 2 direita,esquerda
	corner_points.push_back(cv::Point3f(-0.3301, 0.4307, -0.1 / scalar) * scalar);	// 3 direita,esquerda
	corner_points.push_back(cv::Point3f(-0.3647, 0.4307, -0.1 / scalar) * scalar);	// 4 direita,esquerda
	corner_points.push_back(cv::Point3f(-0.3983, 0.4307, -0.1 / scalar) * scalar);	// 5 direita,esquerda
	corner_points.push_back(cv::Point3f(-0.4329, 0.4307, -0.1 / scalar) * scalar);  // 6 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, 0.3972, -0.1 / scalar) * scalar);  // 7 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, 0.3636, -0.1 / scalar) * scalar);  // 8 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, 0.3290, -0.1 / scalar) * scalar);  // 9 baixo,cima
	corner_points.push_back(cv::Point3f(-0.4329, 0.2955, -0.1 / scalar) * scalar);  // 10 baixo,cima
	corners_points.push_back(corner_points);

	/* INNER SQUARE */
	gate_points_d.push_back(cv::Point3d(-0.3636, -0.3636, -0.1 / scalar) * scalar);
	gate_points_d.push_back(cv::Point3d(0.3636, -0.3636, -0.1 / scalar) * scalar);
	gate_points_d.push_back(cv::Point3d(0.3636, 0.3636, -0.1 / scalar) * scalar);
	gate_points_d.push_back(cv::Point3d(-0.3636, 0.3636, -0.1 / scalar) * scalar);

	this->number_of_gates = number_of_gates;
	for (int i = 0; i < number_of_gates; i++)
	{
		idvec.push_back(0);
	}
	idvec[0] = 20;

	state_gate_id.push_back(Section_State_t{DBL_MAX, 0}); // startup aruco
	state_gate_id.push_back(Section_State_t{DBL_MAX, 5});
	state_gate_id.push_back(Section_State_t{DBL_MAX, 6});
	state_gate_id.push_back(Section_State_t{DBL_MAX, 7});
	state_gate_id.push_back(Section_State_t{DBL_MAX, 57});
	section_state = 0;
}

GateProcessing::~GateProcessing() {}

void GateProcessing::identify(std::vector<std::vector<cv::Point2f>> markerCorners, std::vector<int> markerIds, Mat frame, std::vector<bbox_t> gates_yolo, vector<Gate_t> &identificated_gates, std::vector<int> &idvec, std::vector<bbox_t> &Corners_square, int current_curve)
{
	// Clear array with identificated gates
	identificated_gates.clear();

	// Return if it cannot detect a gate with its aruco id
	if (markerIds.size() == 0 && gates_yolo.size() == 0)
	{
		// ROS_WARN("Couldnt find any aruco or gate.");
		return;
	}

	// Decrease the confidence on all state if it cannot find aruco id
	vector<Aruco_Rect_> markers;
	if (markerIds.size() == 0 && gates_yolo.size() != 0)
	{
		int best_id = 0;
		int best_value = -1;
		for (unsigned int u = 0; u < idvec.size(); u++)
		{
			if (idvec[u] > best_value)
			{
				best_value = idvec[u];
				best_id = u;
			}
		}
		// ROS_WARN("Couldnt find any aruco, but I founded gates. (%d)", best_id);
		for (unsigned int u = 0; u <= best_id; u++)
			idvec[u] = std::max(0, idvec[u] - 1);
	}
	// If it could find a gates and arucos, get the arucos` position
	else
	{
		for (unsigned int i = 0; i < markerIds.size(); i++)
		{
			Aruco_Rect_ a_;
			a_.id = markerIds[i];
			int mx = 0, my = 0;
			mx += (int)markerCorners[i][0].x;
			my += (int)markerCorners[i][0].y;
			mx += (int)markerCorners[i][1].x;
			my += (int)markerCorners[i][1].y;
			mx += (int)markerCorners[i][2].x;
			my += (int)markerCorners[i][2].y;
			mx += (int)markerCorners[i][3].x;
			my += (int)markerCorners[i][3].y;

			cv::Point mean(mx / 4.0, my / 4.0);
			a_.center = mean;
			a_.rect = cv::boundingRect(markerCorners[i]);
			markers.push_back(a_);
			cv::circle(frame, a_.center, 4, Scalar(0, 0, 255), -1, 8, 0);
		}
	}

	// Get bigger Gate
	// For each gate detected on the scene, try to identify it
	int bigger_gate_id = 0;
	int bigger_gate_area = 0;
	for (unsigned int i = 0; i < gates_yolo.size(); i++)
	{
		if (gates_yolo[i].obj_id == 0) // if is a gate
		{
			cv::Rect rect_gate(gates_yolo[i].x, gates_yolo[i].y, gates_yolo[i].w, gates_yolo[i].h);
			float width_c = 640.0 / 2.0;
			float dist_center = (width_c - fabs(width_c - (gates_yolo[i].x + gates_yolo[i].w / 2))) / width_c;
			// ROS_INFO(">>>>> Checking gate: %d dist to center %f", i, dist_center);
			if ((rect_gate.area() * dist_center > bigger_gate_area))
			{
				bigger_gate_area = (int)((float)rect_gate.area() * dist_center);
				bigger_gate_id = i;
			}
		}
	}

	// For each gate detected on the scene, try to identify it
	unsigned int i = bigger_gate_id;

	// Is a boundingbox of a gate, then search for the 4-corner of the gate and sort them, then identify the gate
	if (gates_yolo[i].obj_id == 0)
	{
		// Check list of the founded corners, order by 1,2,3,4
		std::vector<int> corners_founded = {0, 0, 0, 0};
		Gate_t g;
		g.corners_founded.push_back(0);
		g.corners_founded.push_back(0);
		g.corners_founded.push_back(0);
		g.corners_founded.push_back(0);
		g.gate_id = current_curve + 1;
		g.prob = gates_yolo[i].prob;
		/* Avoid gates with low confidence */
		if (g.prob < 0.50)
		{
			ROS_WARN("Discarding gate due high uncertanty detection (%f < 0.50).", g.prob);
		}

		g.x = gates_yolo[i].x;
		g.y = gates_yolo[i].y;
		g.w = gates_yolo[i].w;
		g.h = gates_yolo[i].h;
		cv::Rect rect_gate(gates_yolo[i].x, gates_yolo[i].y, gates_yolo[i].w, gates_yolo[i].h);
		g.rect_gate = rect_gate;
		// Center of the gate
		Point center_gate = Point((gates_yolo[i].x + gates_yolo[i].w / 2), (gates_yolo[i].y + gates_yolo[i].h / 2));

		// Search for the best corners for the detected gate i
		std::vector<bbox_t> best_corners;
		// Change here for exclude bad corners. Corners with 0.7 confidence will be avoided
		g.prob_1 = g.prob_2 = g.prob_3 = g.prob_4 = 0.7;

		// Search for all corners the intercept the gate i
		for (unsigned int j = 0; j < gates_yolo.size(); j++)
		{
			// Is bbox a corner
			if (gates_yolo[j].obj_id == 1)
			{
				cv::Rect rect(gates_yolo[j].x, gates_yolo[j].y, gates_yolo[j].w, gates_yolo[j].h);
				if (((rect_gate & rect).area() > 0))
				{
					best_corners.push_back(gates_yolo[j]);
				}
			}
		}
		Corners_square = best_corners;
		/* Sort gate corners by id 1-2|3-4 */
		if (best_corners.size() >= 3)
		{
			// Draw gate bbox
			for (unsigned int j = 0; j < best_corners.size(); j++)
			{
				cv::Rect rect(best_corners[j].x, best_corners[j].y, best_corners[j].w, best_corners[j].h);
				Point center = Point((best_corners[j].x + best_corners[j].w / 2), (best_corners[j].y + best_corners[j].h / 2));
				float dx = float(center_gate.x - center.x);
				float dy = float(center_gate.y - center.y);
				//float dist = sqrt(dy*dy + dx*dx);
				float alpha = atan2(dy, dx) + CV_PI;
				if (alpha >= 0 && alpha < CV_PI / 2.0)
				{
					if (best_corners[j].prob < g.prob_3)
						continue;
					g.corners_founded[2] = 1;
					g.prob_3 = best_corners[j].prob;
					g.corner_3 = center;
					g.rect_3 = rect;
				}
				else if (alpha >= CV_PI / 2.0 && alpha < CV_PI)
				{
					if (best_corners[j].prob < g.prob_4)
						continue;
					g.corners_founded[3] = 1;
					g.prob_4 = best_corners[j].prob;
					g.corner_4 = center;
					g.rect_4 = rect;
				}
				else if (alpha >= CV_PI && alpha < 3.0 * CV_PI / 2.0)
				{
					if (best_corners[j].prob < g.prob_1)
						continue;
					g.prob_1 = best_corners[j].prob;
					g.corners_founded[0] = 1;
					g.corner_1 = center;
					g.rect_1 = rect;
				}
				else if (alpha >= 3.0 * CV_PI / 2.0 && alpha < 2 * CV_PI)
				{
					if (best_corners[j].prob < g.prob_2)
						continue;
					g.prob_2 = best_corners[j].prob;
					g.corners_founded[1] = 1;
					g.corner_2 = center;
					g.rect_2 = rect;
				}
				cv::circle(frame, center, 1, Scalar(0, 0, 255), -1, 8, 0);
			}

			// Try to identify the detected gate
			// If couldn`t find any aruco, set the gate id with the one with best confidence
			if (markers.size() == 0)
			{
				// ROS_WARN("Couldnt find any aruco, so lets get the one with best confidence. Should this happen?");
				int best_id = 0;
				int best_value = -1;
				for (unsigned int u = 0; u < idvec.size(); u++)
				{
					if (idvec[u] > best_value)
					{
						best_value = idvec[u];
						best_id = u;
					}
				}
				g.gate_id = (current_curve + 1);
				// g.gate_id = (best_id + 1);
			}
			// If it could find aruco near to the gates, then find the one that is closer
			else
			{
				int d_scale_min = INT_MAX;
				// for each aruco, search the one that best identify the gate i
				for (unsigned int m = 0; m < markers.size(); m++)
				{
					cv::putText(frame, to_string(markers[m].id), cv::Point(markers[m].center.x, markers[m].center.y), cv::FONT_HERSHEY_DUPLEX, 1.0, Scalar(0, 0, 255), 2);
					// Aruco 0 - only indicates the start of the circuit
					if (markers[m].id == 0)
					{
						continue;
					}
					if (markers[m].id == 55)
					{
						markers[m].id = 9;
					}


					// Expand the marker until it intersect with the gate i
					// ROS_INFO(">>>>>>>>>>>> Area: %d", rect_gate.area());
					for (int s = 0; s < 200; s++)
					{
						Rect m_r(markers[m].rect);
						m_r = enlargeROI(frame, m_r, s);
						if ((m_r & rect_gate).area() > 0)
						{
							// If the scale factor need to get intersection is larger comparing all detected marker, then this marker is the best one
							if (s < d_scale_min)
							{
								g.gate_id = markers[m].id;
								d_scale_min = s;
							}
							break;
						}
					}

				}
			}
		}

		if (g.gate_id == 9)
		{
			g.gate_id = 55;
		}

		cv::putText(frame, to_string(g.gate_id), cv::Point(g.x + g.w / 2, g.y + g.h / 2), cv::FONT_HERSHEY_DUPLEX, 1.4, Scalar(0, 255, 0), 2);
		cv::putText(frame, to_string_with_precision(g.prob, 2), cv::Point(g.x + g.w / 2, g.y + g.h / 2 + 25), cv::FONT_HERSHEY_DUPLEX, .5, Scalar(0, 255, 0), 1);

		g.corners.push_back(g.corner_1);
		g.corners.push_back(g.corner_2);
		g.corners.push_back(g.corner_3);
		g.corners.push_back(g.corner_4);
		if (g.corners_founded[0] && g.corners_founded[1] && g.corners_founded[2] && g.corners_founded[3])
		{

			identificated_gates.push_back(g);
		}
	}
	// }
}

void GateProcessing::identify(std::vector<std::vector<cv::Point2f>> markerCorners, std::vector<int> markerIds,
							  Mat frame, std::vector<bbox_t> gates_yolo, vector<Gate_t> &identificated_gates)
{
	// SEARCHING ARUCOs
	identificated_gates.clear();
	vector<Aruco_Rect_> markers;

	if (markerCorners.size() == 0 && gates_yolo.size() == 0)
	{
		return;
	}
	else if (markerCorners.size() == 0 && gates_yolo.size() != 0)
	{
		if (idvec[0] > 0)
		{
			idvec[0] = idvec[0] - 1;
		}
		if (idvec[1] > 0)
		{
			idvec[1] = idvec[1] - 1;
		}
		if (idvec[2] > 0)
		{
			idvec[2] = idvec[2] - 1;
		}
		if (idvec[3] > 0)
		{
			idvec[3] = idvec[3] - 1;
		}
	}
	else
	{
		for (unsigned int i = 0; i < markerIds.size(); i++)
		{
			Aruco_Rect_ a_;
			a_.id = markerIds[i];
			int mx = 0, my = 0;
			mx += (int)markerCorners[i][0].x;
			my += (int)markerCorners[i][0].y;
			mx += (int)markerCorners[i][1].x;
			my += (int)markerCorners[i][1].y;
			mx += (int)markerCorners[i][2].x;
			my += (int)markerCorners[i][2].y;
			mx += (int)markerCorners[i][3].x;
			my += (int)markerCorners[i][3].y;

			cv::Point mean(mx / 4, my / 4);
			a_.center = mean;
			a_.rect = cv::boundingRect(markerCorners[i]);
			markers.push_back(a_);
			//			  cv::circle(frame, a_.center, 4, Scalar(0, 255, 0), -1, 8, 0);
		}
	}

	for (unsigned int i = 0; i < gates_yolo.size(); i++)
	{
		if (gates_yolo[i].obj_id == 0)
		{ // is a boundingbox of a gate
			Gate_t g;
			g.corners_founded.push_back(0);
			g.corners_founded.push_back(0);
			g.corners_founded.push_back(0);
			g.corners_founded.push_back(0);
			g.gate_id = -1;
			g.prob = gates_yolo[i].prob;
			if (g.prob < 0.50)
				continue;
			g.has_pose = false;
			g.x = gates_yolo[i].x;
			g.y = gates_yolo[i].y;
			g.w = gates_yolo[i].w;
			g.h = gates_yolo[i].h;
			cv::Rect rect_gate(gates_yolo[i].x, gates_yolo[i].y, gates_yolo[i].w, gates_yolo[i].h);
			g.rect_gate = rect_gate;
			//				cv::rectangle(frame, rect_gate, cv::Scalar(255, 0, 0));
			Point center_gate = Point((gates_yolo[i].x + gates_yolo[i].w / 2), (gates_yolo[i].y + gates_yolo[i].h / 2));

			std::vector<bbox_t> best_corners;

			for (unsigned int j = 0; j < gates_yolo.size(); j++)
			{ // search the best corners to this gate i
				if (gates_yolo[j].obj_id == 1)
				{
					cv::Rect rect(gates_yolo[j].x, gates_yolo[j].y, gates_yolo[j].w, gates_yolo[j].h);
					//Point center = Point((gates_yolo[j].x + gates_yolo[j].w / 2), (gates_yolo[j].y + gates_yolo[j].h / 2));
					if (((rect_gate & rect).area() > 0))
					{
						best_corners.push_back(gates_yolo[j]);
					}
				}
			}

			g.prob_1 = g.prob_2 = g.prob_3 = g.prob_4 = 0.7; //// MUDAR AQUI PARA EXCLUIR CORNERS RUIMS

			if (best_corners.size() >= 3)
			{
				for (unsigned int j = 0; j < best_corners.size(); j++)
				{
					cv::Rect rect(best_corners[j].x, best_corners[j].y, best_corners[j].w, best_corners[j].h);
					Point center = Point((best_corners[j].x + best_corners[j].w / 2), (best_corners[j].y + best_corners[j].h / 2));
					float dx = float(center_gate.x - center.x);
					float dy = float(center_gate.y - center.y);
					//float dist = sqrt(dy*dy + dx*dx);
					float alpha = atan2(dy, dx) + CV_PI;
					if (alpha >= 0 && alpha < CV_PI / 2.0)
					{
						if (best_corners[j].prob < g.prob_3)
							continue;
						g.corners_founded[2] = 1;
						g.prob_3 = best_corners[j].prob;
						g.corner_3 = center;
						g.rect_3 = rect;
						//						cv::putText(frame, "3", cv::Point(center.x, center.y-5), cv::FONT_HERSHEY_DUPLEX, 1.0, Scalar(0, 255, 0), 2);
						//						cv::putText(frame, to_string(best_corners[j].prob), cv::Point(center.x, center.y+5), cv::FONT_HERSHEY_DUPLEX, .5, Scalar(0, 255, 0), 1);
					}
					else if (alpha >= CV_PI / 2.0 && alpha < CV_PI)
					{
						if (best_corners[j].prob < g.prob_4)
							continue;
						g.corners_founded[3] = 1;
						g.prob_4 = best_corners[j].prob;
						g.corner_4 = center;
						g.rect_4 = rect;
						//						cv::putText(frame, "4", cv::Point(center.x, center.y-5), cv::FONT_HERSHEY_DUPLEX, 1.0, Scalar(0, 255, 0), 2);
						//						cv::putText(frame, to_string(best_corners[j].prob), cv::Point(center.x, center.y+5), cv::FONT_HERSHEY_DUPLEX, .5, Scalar(0, 255, 0), 1);
					}
					else if (alpha >= CV_PI && alpha < 3.0 * CV_PI / 2.0)
					{
						if (best_corners[j].prob < g.prob_1)
							continue;
						g.prob_1 = best_corners[j].prob;
						g.corners_founded[0] = 1;
						g.corner_1 = center;
						g.rect_1 = rect;
						//						cv::putText(frame, "1", cv::Point(center.x, center.y-5), cv::FONT_HERSHEY_DUPLEX, 1.0, Scalar(0, 255, 0), 2);
						//cv::putText(frame, to_string(best_corners[j].prob), cv::Point(center.x, center.y+5), cv::FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0), 1);
					}
					else if (alpha >= 3.0 * CV_PI / 2.0 && alpha < 2 * CV_PI)
					{
						if (best_corners[j].prob < g.prob_2)
							continue;
						g.prob_2 = best_corners[j].prob;
						g.corners_founded[1] = 1;
						g.corner_2 = center;
						g.rect_2 = rect;
						//cv::putText(frame, "2", cv::Point(center.x, center.y-5), cv::FONT_HERSHEY_DUPLEX, 1.0, Scalar(0, 255, 0), 2);
						//cv::putText(frame, to_string(best_corners[j].prob), cv::Point(center.x, center.y+5), cv::FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0), 1);
					}
					//cv::circle(frame, center, 4, Scalar(0, 0, 255), -1, 8, 0);
					//cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);
				}

				//				  static bool flag1 = false;
				//				  static bool flag2 = false;
				//				  static bool flag3 = false;
				int d_scale_min = INT_MAX;
				if (markers.size() == 0)
				{
					//					  if((idvec[0] > idvec[1] || idvec[0] > idvec[2] || idvec[0] > idvec[3]) && flag1==false && flag3==false && flag3==false){
					if (idvec[0] > idvec[1] || idvec[0] > idvec[2] || idvec[0] > idvec[3])
					{
						g.gate_id = 5;
					}
					//					  else if ((idvec[1] > idvec[0] || idvec[1] > idvec[2] || idvec[1] > idvec[3]) && flag3==false && flag3==false){
					else if (idvec[1] > idvec[0] || idvec[1] > idvec[2] || idvec[1] > idvec[3])
					{
						//							flag1 = true;
						g.gate_id = 6;
					}
					//					  else if ((idvec[2] > idvec[1] || idvec[2] > idvec[0] || idvec[2] > idvec[3]) && flag1==true && flag3==false){
					else if (idvec[2] > idvec[1] || idvec[2] > idvec[0] || idvec[2] > idvec[3])
					{
						//							flag2 = true;
						g.gate_id = 7;
					}
					//					  else if ((idvec[3] > idvec[1] || idvec[3] > idvec[2] || idvec[3] > idvec[0]) && flag1==true  && flag2==true ){
					else if (idvec[3] > idvec[1] || idvec[3] > idvec[2] || idvec[3] > idvec[0])
					{
						//							flag3 = true;
						g.gate_id = 57;
					}
				}
				else
				{

					for (unsigned int m = 0; m < markers.size(); m++)
					{
						//cv::putText(frame, to_string(markers[m].id), cv::Point(markers[m].center.x, markers[m].center.y), cv::FONT_HERSHEY_DUPLEX, .5, Scalar(0, 255, 0), 2);
						if (markers[m].id != 5 && markers[m].id != 6 && markers[m].id != 7 && markers[m].id != 57)
						{
							if (idvec[0] > idvec[1] + 2 || idvec[0] > idvec[2] + 2 || idvec[0] > idvec[3] + 2)
							{
								g.gate_id = 5;
							}
							else if (idvec[1] > idvec[0] + 2 || idvec[1] > idvec[2] + 2 || idvec[1] > idvec[3] + 2)
							{
								g.gate_id = 6;
							}
							else if (idvec[2] > idvec[1] + 2 || idvec[2] > idvec[0] + 2 || idvec[2] > idvec[3] + 2)
							{
								g.gate_id = 7;
							}
							else if (idvec[3] > idvec[1] + 2 || idvec[3] > idvec[2] + 2 || idvec[3] > idvec[0] + 2)
							{
								g.gate_id = 57;
							}
							else
							{
								g.gate_id = 14;
								continue;
							}
						}

						for (int s = 0; s < 200; s++)
						{
							Rect m_r(markers[m].rect);
							m_r = enlargeROI(frame, m_r, s);
							if ((m_r & rect_gate).area() > 0)
							{
								if (s < d_scale_min)
								{
									g.gate_id = markers[m].id;
									d_scale_min = s;
									if (g.gate_id == 5)
									{
										idvec[0] = idvec[0] + 1;
										if (idvec[1] > 0)
										{
											idvec[1] = idvec[1] - 1;
										}
										if (idvec[2] > 0)
										{
											idvec[2] = idvec[2] - 1;
										}
										if (idvec[3] > 0)
										{
											idvec[3] = idvec[3] - 1;
										}
									}
									else if (g.gate_id == 6)
									{
										idvec[1] = idvec[1] + 1;
										if (idvec[0] > 0)
										{
											idvec[0] = idvec[0] - 1;
											if (idvec[0] > 0)
											{
												idvec[0] = idvec[0] - 1;
											}
										}
										if (idvec[2] > 0)
										{
											idvec[2] = idvec[2] - 1;
										}
										if (idvec[3] > 0)
										{
											idvec[3] = idvec[3] - 1;
										}
									}
									else if (g.gate_id == 7)
									{
										idvec[2] = idvec[2] + 1;
										if (idvec[0] > 0)
										{
											idvec[0] = idvec[0] - 1;
											if (idvec[0] > 0)
											{
												idvec[0] = idvec[0] - 1;
											}
										}
										if (idvec[1] > 0)
										{
											idvec[1] = idvec[0] - 1;
											if (idvec[1] > 0)
											{
												idvec[1] = idvec[0] - 1;
											}
										}
										if (idvec[3] > 0)
										{
											idvec[3] = idvec[3] - 1;
										}
									}
									else if (g.gate_id == 57)
									{
										idvec[3] = idvec[3] + 1;
										if (idvec[0] > 0)
										{
											idvec[0] = idvec[0] - 1;
											if (idvec[0] > 0)
											{
												idvec[0] = idvec[0] - 1;
											}
										}
										if (idvec[2] > 0)
										{
											idvec[2] = idvec[2] - 1;
											if (idvec[2] > 0)
											{
												idvec[2] = idvec[2] - 1;
											}
										}
										if (idvec[1] > 0)
										{
											idvec[1] = idvec[1] - 1;
											if (idvec[1] > 0)
											{
												idvec[1] = idvec[0] - 1;
											}
										}
									}
								}
								break;
							}
						}
					}
				}
			}

			g.corners.push_back(g.corner_1);
			g.corners.push_back(g.corner_2);
			g.corners.push_back(g.corner_3);
			g.corners.push_back(g.corner_4);

			if (g.corners_founded[0] && g.corners_founded[1] && g.corners_founded[2] && g.corners_founded[3]){ // melhorar aqui
				identificated_gates.push_back(g);
			}
		}
	}
}




std::tuple< cv::Mat, std::vector<xquad_autonomous_stack::ArucoArray> > GateProcessing::getPose(cv::Mat frame, vector<Gate_t> &identificated_gates, cv::Mat intrinsic_matrix, cv::Mat distortion_coeff)
{


	PyObject *pName, *pModule, *pFunc;

	// Aruco para aproveitar a msg
	xquad_autonomous_stack::ArucoArray cornersPoints_vect;
	std::vector<xquad_autonomous_stack::ArucoArray> gate_corners;
	


	static int counter_img = 0;
	static bool flag_ransac = false;

	int number_points = 0;

	for (unsigned int i = 0; i < identificated_gates.size(); i++)
	{
		Gate_t g = identificated_gates[i];
		cv::Point2f center(g.rect_gate.x + g.rect_gate.width / 2, g.rect_gate.y + g.rect_gate.height / 2);

		std::vector<cv::Rect> rois;

		rois.push_back(g.rect_1);
		rois.push_back(g.rect_2);
		rois.push_back(g.rect_3);
		rois.push_back(g.rect_4);

		std::vector<cv::Point3f> gate_points_;
		std::vector<cv::Point2f> image_points_;

		for (unsigned int j = 0; j < g.corners.size(); j++)
		{
			if (!g.corners_founded[j])
				continue;
			if (rois[j].width == 0 || rois[j].height == 0)
				continue;
			if ((rois[j].x + rois[j].width) > frame.size().width || (rois[j].y + rois[j].height) > frame.size().height)
				continue;
			rois[j] = enlargeROI(frame, rois[j], 4);
			cv::Mat frame_roi = frame(rois[j]);
			cv::erode(frame_roi, frame_roi, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
			// cv::rectangle(frame, rois[j], cv::Scalar(0, 255, 0));
			cbdetect::Corner corners;
			std::vector<cv::Point2f> sorted;
			cbdetect::Params params;
			params.corner_type = cbdetect::SaddlePoint;
			params.norm_half_kernel_size = 31;

			cbdetect::find_corners(frame_roi, corners, params);

			// Fix over than 10 points
			if (corners.p.size() > 10)
			{
				flag_ransac = false;
				std::vector<double> dist_mean;
				for (unsigned int k = 0; k < corners.p.size(); k++)
				{
					double dist_k = DBL_MAX;
					double dist_k_l = DBL_MAX;
					for (unsigned int l = 0; l < corners.p.size(); l++)
					{
						if (k == l)
							continue;
						double dx = corners.p[k].x - corners.p[l].x;
						double dy = corners.p[k].y - corners.p[l].y;
						double dist = sqrt(dx * dx + dy * dy);
						if (dist < dist_k)
						{
							dist_k_l = dist_k;
							dist_k = dist;
						}
						else if (dist < dist_k_l)
						{
							dist_k_l = dist;
						}
					}
					dist_mean.push_back((dist_k + dist_k_l));
				}

				while (corners.p.size() > 10)
				{
					int index = std::max_element(dist_mean.begin(), dist_mean.end()) - dist_mean.begin();
					corners.p.erase(corners.p.begin() + index);
					dist_mean.erase(dist_mean.begin() + index);
				}
			}

		


			// cout<<"corners size:"<<corners.p.size()<<endl;
			if (corners.p.size() < 6)
			{
				// cout <<"Não tem 10, descartou tudo!\n";
				continue;
			}



			// Convert corner from frame_roi to frame
			for (unsigned int k = 0; k < corners.p.size(); k++)
			{
				corners.p[k].x += rois[j].x;
				corners.p[k].y += rois[j].y;
				if (corners.p.size() == 10)
				{
					cv::circle(frame, corners.p[k], 2, cv::Scalar(255, 0, 0), -1);
				}
				else
				{
					cv::circle(frame, corners.p[k], 2, cv::Scalar(0, 255, 255), -1);
				}
			}

			cv::circle(frame, center, 1, cv::Scalar(0, 255, 0), -1);
			cv::putText(frame, std::to_string(g.gate_id), cv::Point2i(center.x, center.y - 12),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 0, 255), 2);


						
				
				

			// Collect id of the small squares (corners of the big gate)
			xquad_autonomous_stack::Aruco cPoints; // Aruco para aproveitar a msg
			cPoints.id = j;
			rtabmap_ros::Point2f corners_aux;
			for (unsigned int k = 0; k < corners.p.size(); k++)
			{	
				// Save pixels values of each dot inside of the small square.
				corners_aux.x = corners.p[k].x;
				corners_aux.y = corners.p[k].y;
				cPoints.corners.push_back(corners_aux);


				sorted.push_back(corners.p[k]);
				gate_points_.push_back(corners_points[j][k]);
				image_points_.push_back(corners.p[k]);
				//cv::putText(frame, std::to_string(k), cv::Point2i(corners.p[k].x - 12, corners.p[k].y - 12),
				//		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
			}

			// Save all corner points of the i-gate
			cornersPoints_vect.aruco.push_back(cPoints);


			// Include corner points
			gate_points_.push_back(gate_points_d[0]);
			gate_points_.push_back(gate_points_d[1]);
			gate_points_.push_back(gate_points_d[2]);
			gate_points_.push_back(gate_points_d[3]);
			image_points_.push_back(cv::Point2f(g.corner_1.x,g.corner_1.y));
			image_points_.push_back(cv::Point2f(g.corner_2.x,g.corner_2.y));
			image_points_.push_back(cv::Point2f(g.corner_3.x,g.corner_3.y));
			image_points_.push_back(cv::Point2f(g.corner_4.x,g.corner_4.y));

		}


		// ADD Yolo Corners
		xquad_autonomous_stack::Aruco cPoints;
		cPoints.id = 100;
		rtabmap_ros::Point2f corners_aux;
		for (unsigned int k = 0; k < 4; k++)
		{
			// cout<<"g.corners[k]: "<<g.corners[k].x<<endl;
			corners_aux.x = g.corners[k].x;
			corners_aux.y = g.corners[k].y;
			cPoints.corners.push_back(corners_aux);
		}
		cornersPoints_vect.aruco.push_back(cPoints);

		// Save all gates corners
		gate_corners.push_back(cornersPoints_vect);


		g.rvec = cv::Mat(3, 1, cv::DataType<double>::type);
		g.tvec = cv::Mat(3, 1, cv::DataType<double>::type);


		number_points = image_points_.size();
		if (image_points_.size() < 20){
			continue; 
		}
			

		//cout << "GatePoints: " << gate_points_.size() << " Sorted Size: " << image_points_.size() << endl;
		if (gate_points_.size() == 0 || image_points_.size() == 0)
			continue;
		Mat distortion_coeff2 = (cv::Mat_<float>(1, 5) << 0, 0, 0., 0., 0); // image is already undistorted
		// SOLVE PNP
		if (flag_ransac)
		{
			cv::solvePnPRansac(gate_points_, image_points_, intrinsic_matrix, distortion_coeff2, g.rvec, g.tvec, SOLVEPNP_ITERATIVE);
		}
		else
		{
			cv::solvePnP(gate_points_, image_points_, intrinsic_matrix, distortion_coeff2, g.rvec, g.tvec, false, SOLVEPNP_ITERATIVE);
			//cout<<"PNP TYPE: ITERATIVE\t"<<is_ransac<<endl;
		}

		g.has_pose = true;
		double dist_cam_to_gate = sqrt(g.tvec.at<double>(0) * g.tvec.at<double>(0) +
									   g.tvec.at<double>(1) * g.tvec.at<double>(1) +
									   g.tvec.at<double>(2) * g.tvec.at<double>(2));


		identificated_gates[i] = g;

		// Gate Projection
		std::vector<cv::Point2d> imagePoints;
		cv::projectPoints(gate_points_d, g.rvec, g.tvec, intrinsic_matrix, distortion_coeff2, imagePoints);

		for (unsigned int k = 0; k < 4; k++)
		{
			cv::Point p1(imagePoints[k].x, imagePoints[k].y);
			cv::Point p2(imagePoints[(k + 1) % 4].x, imagePoints[(k + 1) % 4].y);
			//std::cout << "points projeted" << p1 << p2 << std::endl;
			cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
			// cv::circle(frame, g.corners[k], 2, cv::Scalar(0, 255, 0), -1);
			cv::circle(frame, imagePoints[k], 2, cv::Scalar(0, 255, 0), -1);
		}
	}


	return std::make_tuple( frame, gate_corners);
}

Rect GateProcessing::enlargeROI(Mat frm, Rect boundingBox, int padding)
{
	Rect returnRect = Rect(boundingBox.x - padding, boundingBox.y - padding,
						   boundingBox.width + (padding * 2), boundingBox.height + (padding * 2));
	if (returnRect.x < 0)
		returnRect.x = 0;
	if (returnRect.y < 0)
		returnRect.y = 0;
	if (returnRect.x + returnRect.width >= frm.cols)
		returnRect.width = frm.cols - returnRect.x;
	if (returnRect.y + returnRect.height >= frm.rows)
		returnRect.height = frm.rows - returnRect.y;
	return returnRect;
}

#include "windows_player.h"

PlayerWindows::PlayerWindows()
{
	m_status = true;
	m_imgWidth = 752;
	m_imgHeight = 480;

	m_tic.setZero();
	m_ric.setIdentity();
	m_fre.setZero();

	m_loopCorrectT.setZero();
	m_loopCorrectR.setIdentity();

	m_curPosT.setZero();
	m_curPosR.setIdentity();

	m_featureImg = cv::Mat(m_imgHeight, m_imgWidth, CV_8UC3, 0);

	float mViewpointX = -0;
	float mViewpointY = -5;
	float mViewpointZ = -10;
	float mViewpointF = 500;

	pangolin::CreateWindowAndBind("VINS: Map Visualization", 1024, 768);
	glEnable(GL_DEPTH_TEST);
	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, float(300.0 / 1024.0));
	pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
	pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
	pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);

	//m_menuFollowCamera = std::move(menuFollowCamera);
	//m_menuShowPoints = std::move(menuShowPoints);
	//m_menuShowPath = std::move(menuShowPath);

	std::vector<std::string> labelTic;
	std::vector<std::string> labelRic;
	std::vector<std::string> labelFre;

	labelTic.emplace_back(std::string("Tic.x"));
	labelTic.emplace_back(std::string("Tic.y"));
	labelTic.emplace_back(std::string("Tic.z"));

	labelRic.emplace_back(std::string("yaw"));
	labelRic.emplace_back(std::string("pitch"));
	labelRic.emplace_back(std::string("roll"));

	labelFre.emplace_back(std::string("feature_freq"));
	labelFre.emplace_back(std::string("process_freq"));
	labelFre.emplace_back(std::string("player_freq"));

	m_logTic.SetLabels(labelTic);
	m_logRic.SetLabels(labelRic);
	m_logFre.SetLabels(labelFre);

	pangolin::Plotter plotterRic(&m_logRic, 0.0f, 100.0f, -0.02f, 0.02f, 10.0f, 0.001f);
	plotterRic.SetBounds(float(240.0 / 768.0), float(440.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));
	plotterRic.Track("$i");

	pangolin::Plotter plotterTic(&m_logTic, 0.0f, 100.0f, -0.02f, 0.02f, 10.0f, 0.001f);
	plotterTic.SetBounds(float(20.0 / 768.0), float(220.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));
	plotterTic.Track("$i");

	pangolin::Plotter plotterFre(&m_logFre, 0.0f, 100.0f, 0.0f, 20.0f, 10.0f, 0.5f);
	plotterFre.SetBounds(float(460.0 / 768.0), float(660.0 / 768.0), float(10.0 / 1024.0), float(290.0 / 1024.0));
	plotterFre.Track("$i");

	pangolin::DisplayBase().AddDisplay(plotterRic);
	pangolin::DisplayBase().AddDisplay(plotterTic);
	pangolin::DisplayBase().AddDisplay(plotterFre);

	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
		pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 1, 0, 1)
	);
	m_s_camera = std::move(s_cam);

	pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, float(300.0 / 1024.0), 1.0, -1024.0f / 768.0f)
		.SetHandler(new pangolin::Handler3D(s_cam));
	m_d_camera = std::move(d_cam);

	float ratio = float(m_imgHeight) / float(m_imgWidth);
	int img_width = 280;
	int img_height = 280.0*ratio;
	pangolin::View& d_image = pangolin::CreateDisplay()
		.SetBounds(float(768 - img_height) / 768.0, 1.0f, float(300.0 / 1024.0), float(300 + img_width) / 1024.0, float(img_width) / img_height)
		.SetLock(pangolin::LockLeft, pangolin::LockTop);
	m_d_image = std::move(d_image);

	unsigned char* imgArray = new unsigned char[3 * m_imgHeight*m_imgWidth];
	pangolin::GlTexture imageTexture(m_imgWidth, m_imgHeight, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
	m_imgTexture = std::move(imageTexture);
}

PlayerWindows::~PlayerWindows()
{

}

void PlayerWindows::close()
{
	m_status = false;
}

void PlayerWindows::play()
{
	pangolin::OpenGlMatrix Twc;
	Twc.SetIdentity();
	Eigen::Vector3d m_ypr;
	m_ypr.setZero();

	while (!pangolin::ShouldQuit() && m_status)
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(1.0f, 1.0f, 1.0f, 0.5f);

		ViewCameraPose(Twc);
		R2ypr(m_ric, m_ypr);

		m_logRic.Log(m_ypr(0), m_ypr(1), m_ypr(2));
		m_logTic.Log(m_tic(0), m_tic(1), m_tic(2));
		m_logFre.Log(m_fre(0), m_fre(1), m_fre(2));

		m_s_camera.Follow(Twc);
		m_d_camera.Activate(m_s_camera);
		DrawCurrentCamera(Twc);

		ViewCameraPath();
		ViewCameraLocalLandmarks();
		ViewCameraGlobalLandmark();

		m_d_image.Activate();
		memcpy(m_imgArray, m_featureImg.data,
			sizeof(uchar) * 3 * m_imgHeight*m_imgWidth);
		m_imgTexture.Upload(m_imgArray, GL_RGB, GL_UNSIGNED_BYTE);
		m_imgTexture.RenderToViewport();
		
		pangolin::FinishFrame();
	}
}

void PlayerWindows::ViewCameraPath()
{
	GLfloat line_width = 1.2;
	Eigen::Vector3d tmp_path;
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(line_width);
	glBegin(GL_LINE_STRIP);

	for (const auto& it : m_keypose) {
		glVertex3f(it.x(), it.y(), it.z());
	}
	glEnd();
}

void PlayerWindows::ViewCameraGlobalLandmark()
{
	glPointSize(1.5f);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 1.0);
	for (const auto& pairs : m_localLandmarks) {
		const auto& landmark = pairs.second;
		glVertex3f(landmark.x(), landmark.y(), landmark.z());
	}
	glEnd();
}

void PlayerWindows::ViewCameraLocalLandmarks()
{
	glPointSize(1.5f);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);
	for (const auto& pairs : m_globalLandmarks) {
		const auto& landmark = pairs.second;
		glVertex3f(landmark.x(), landmark.y(), landmark.z());
	}
	glEnd();
}

void PlayerWindows::ViewCameraPose(pangolin::OpenGlMatrix& M)
{
	Eigen::Vector3d P = (m_loopCorrectR * m_curPosT + m_loopCorrectT) +
		(m_loopCorrectR * m_curPosR) * m_tic;
	Eigen::Matrix3d R = (m_loopCorrectR * m_curPosR) * m_ric;

	M.m[0] = R(0, 0);
	M.m[1] = R(1, 0);
	M.m[2] = R(2, 0);
	M.m[3] = 0.0;

	M.m[4] = R(0, 1);
	M.m[5] = R(1, 1);
	M.m[6] = R(2, 1);
	M.m[7] = 0.0;

	M.m[8] = R(0, 2);
	M.m[9] = R(1, 2);
	M.m[10] = R(2, 2);
	M.m[11] = 0.0;


	M.m[12] = P.x();
	M.m[13] = P.y();
	M.m[14] = P.z();
	M.m[15] = 1.0;
}

void PlayerWindows::DrawCurrentCamera(pangolin::OpenGlMatrix& Twc)
{
	const float &w = 0.08f;
	const float h = w * 0.75;
	const float z = w * 0.6;

	glPushMatrix();

#ifdef HAVE_GLES
	glMultMatrixf(Twc.m);
#else
	glMultMatrixd(Twc.m);
#endif

	glLineWidth(2);		
	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(w, h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(w, -h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(-w, -h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(-w, h, z);

	glVertex3f(w, h, z);
	glVertex3f(w, -h, z);
	glVertex3f(-w, h, z);
	glVertex3f(-w, -h, z);
	glVertex3f(-w, h, z);
	glVertex3f(w, h, z);
	glVertex3f(-w, -h, z);
	glVertex3f(w, -h, z);
	glEnd();
	glPopMatrix();
}
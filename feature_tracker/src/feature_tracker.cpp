#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

static void drawArrows(cv::Mat& frame, cv::Point2f prevPt, cv::Point2f nextPt, cv::Scalar line_color = cv::Scalar(0, 255,0))
{
  int line_thickness = 2;
  cv::Point p = prevPt;
  cv::Point q = nextPt;

  double angle = atan2((double) p.y - q.y, (double) p.x - q.x);
  double hypotenuse = sqrt( (double)(p.y - q.y)*(p.y - q.y) + (double)(p.x - q.x)*(p.x - q.x) );
  if (hypotenuse < 1.0)
    return;

  // Here we lengthen the arrow by a factor of three.
  //q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
  //q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

  // Now we draw the main line of the arrow.
  cv::line(frame, p, q, line_color, line_thickness);
  //return;
  // Now draw the tips of the arrow. I do some scaling so that the
  // tips look proportional to the main line of the arrow.

  p.x = (int) (q.x + 4 * cos(angle + CV_PI / 4));
  p.y = (int) (q.y + 4 * sin(angle + CV_PI / 4));
  cv::line(frame, p, q, line_color, line_thickness);

  p.x = (int) (q.x + 4 * cos(angle - CV_PI / 4));
  p.y = (int) (q.y + 4 * sin(angle - CV_PI / 4));
  cv::line(frame, p, q, line_color, line_thickness);
}

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_r;

    int i = 9;
    cv::Mat _img1;
    //bilateralFilter(_img, _img1, i, i*2, i/2);
    _img1 = _img.clone();    

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img1, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img1;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
	//cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 3, 1));

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

	cout<<"prev:"<<prev_pts.size()<<endl;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(prev_ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());

#if 1	
	cv::Mat show_img;
	cv::cvtColor(forw_img, show_img, CV_GRAY2RGB);
	for (unsigned int j = 0; j < cur_pts.size(); j++) {
	  cv::circle(show_img, forw_pts[j], 2, cv::Scalar(255,0,0), 2);
	  cv::line(show_img, forw_pts[j], cur_pts[j], cv::Scalar(0, 255, 0), 2);
	}
	cv::imshow("origin track", show_img);
	cv::waitKey(1);
#endif
    }

    if (PUB_THIS_FRAME)
    {
      //1.reject with ORB
      //rejectWithORB();

      //2.reject with F
      rejectWithF();

#if 1
      cv::Mat show_img;
      cv::cvtColor(forw_img, show_img, CV_GRAY2RGB);
      for (unsigned int j = 0; j < forw_pts.size(); j++) {
	for (unsigned int k = 0; k < prev_pts.size(); k++) {
	  if(ids[j] == prev_ids[k]) {
	    char name[10];
	    sprintf(name, "%d", track_cnt[j]);
	    //cv::putText(show_img, name, forw_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0));
	    double len = std::min(1.0, 1.0 * track_cnt[j] / WINDOW_SIZE);
	    //cv::line(show_img, forw_pts[j], prev_pts[k], cv::Scalar(0, 255, 0), 2);
	    drawArrows(show_img,forw_pts[j],prev_pts[j]);
	    cv::circle(show_img, forw_pts[j], 2,cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
	  }
	}
      }
      cv::imshow("track with reject", show_img);
      cv::waitKey(1);
#endif


        for (auto &n : track_cnt)
            n++;

        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.1, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());

	//update id
	for(int i=0;i<ids.size();i++) {
          if (ids[i] == -1)
            ids[i] = n_id++;
        }

        prev_img = forw_img;
        prev_pts = forw_pts;
	prev_ids = ids;
    }
    
    cur_img = forw_img;
    cur_pts = forw_pts;
}

void FeatureTracker::rejectWithF()
{
#if 0
  //test
  Eigen::Vector3d tmpp1,tmpp2;
  m_camera->liftProjective(Eigen::Vector2d(0, 0), tmpp1);
  m_camera->liftProjective(Eigen::Vector2d(640, 480), tmpp2);
  cout<<"lefttop:"<<tmpp1.transpose()<<endl;
  cout<<"rightbottom:"<<tmpp2.transpose()<<endl;
  cout<<"cosA:"<<tmpp1.dot(tmpp2)/tmpp1.norm()/tmpp2.norm()<<endl;
#endif

    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
	reduceVector(prev_ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::rejectWithORB()
{
  ROS_DEBUG("reject with ORB begins");

#if 0
  //prev:prev_pts
  //curr:forw_pts
  TicToc t_f;

  //计算orb描述子并匹配
  Ptr<ORB> orb = ORB::create();
  vector<KeyPoint> keyPoints_1, keyPoints_2;
  Mat descriptors_1, descriptors_2;
  KeyPoint::convert(prev_pts, keyPoints_1);
  KeyPoint::convert(forw_nextPts, keyPoints_2);
  orb->compute(prev_img, keyPoints_1, descriptors_1);
  orb->compute(forw_img, keyPoints_2, descriptors_2);

  BFMatcher matcher;
  vector<DMatch> matches;
  matcher.match(descriptors_1, descriptors_2, matches);

  //计算最大最小距离
  double max_dist = 0; double min_dist = 0; 
  double sum_dist = 0; double mean_dist = 0;
  for( int i = 0; i < descriptors_1.rows; i++ ){ 
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
    sum_dist += dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
  if(descriptors_1.rows > 0) {
    mean_dist = sum_dist/descriptors_1.rows;
    printf("-- Mean dist : %f \n", mean_dist);
  }

  vector<cv::Point2f> pts1;
  vector<cv::Point2f> nextPts1;
  vector<unsigned char> status1;

  //保留好的点
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_1.rows; i++ ) { 
    if( matches[i].distance > mean_dist )  { 
      status[i] = 0;
    }
  }


  reduceVector(prev_pts, status);
  reduceVector(cur_pts, status);
  reduceVector(forw_pts, status);
  reduceVector(ids, status);
  reduceVector(prev_ids, status);
  reduceVector(track_cnt, status);
  ROS_DEBUG("reject with ORB: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
  ROS_DEBUG("reject with ORB costs: %fms", t_f.toc());
#endif
}


bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
	  ids[i] = n_id++;
	  //ids[i] = ++n_id;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
#if 1
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
#else
    cv::Mat undistortedImg(ROW , COL , CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    FOCAL_LENGTH = 350;
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) >= 0 && pp.at<float>(1, 0)  < ROW  && pp.at<float>(0, 0) >= 0 && pp.at<float>(0, 0) < COL)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) , pp.at<float>(0, 0) ) = cur_img.at<uchar>(odistortedp[i].y(), distortedp[i].x());
        }
        //else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
#endif
    //cv::imshow(name, undistortedImg);
    //cv::waitKey(1);
}

vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_pts;
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}

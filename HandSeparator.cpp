#include "HandSeparator.h"
#include <queue>

using namespace cv;
using namespace std;

HandSeparator::HandSeparator(const Mat &depth, const Mat &image, int y, int x)
	:depth(depth), image(image), handPositionY(y), handPositionX(x)
{
}

void HandSeparator::separate(cv::Mat &result)
{
	int y = handPositionY;
	int x = handPositionX;
	queue<pair<int,int> > q;
	Mat flags = Mat::zeros(480, 640, CV_8UC1);
	q.push(make_pair(y, x));
	int dx[] = {1, -1, 0, 0}, dy[]={0, 0, 1, -1};
	int xx, yy;
	int gx1 = x - 10, gx2 = x + 10, gy1 = y - 10, gy2 = y + 10;
	if (gx1 < 0) gx1 = 0;
	if (gy1 < 0) gy1 = 0;
	if (gx2 >= 640) gx1 = 640 - 1;
	if (gy2 >= 480) gy1 = 480 - 1;
	const int inf  = (int)1e8;
	int tx = x, ty = y, mn = inf;
	for (int i = gy1; i <= gy2; i++)
		for (int j = gx1; j <= gx2; j++)
			if (depth.at<unsigned short>(i, j) < mn)
			{
				if (depth.at<unsigned short>(i, j) < 500)continue;
				ty = i;
				tx = j;
				mn = depth.at<unsigned short>(i, j);
			}
	if (mn < 500 || mn > 4000)return;
	x = tx;
	y = ty;
	unsigned short cnDist = depth.at<unsigned short>(y, x);
	unsigned short threshold = 100;
	flags.at<uchar>(y, x) = 1;
	int xmax = -1, xmin = inf, ymax = -1, ymin = inf;
	while (!q.empty())
	{
		pair<int,int> t = q.front();
		q.pop();
		xmax = max(xmax, t.second);
		xmin = min(xmin, t.second);

		ymax = max(ymax, t.first);
		ymin = min(ymin, t.first);
		for (int i = 0; i < 4; i++)
		{
			xx = t.second + dx[i];
			yy = t.first + dy[i];
			if (yy < 0 || xx < 0 || xx >= 640 || yy >= 480)continue;
			if (flags.at<uchar>(yy, xx))continue;
			if (abs(depth.at<unsigned short>(yy, xx) - cnDist) > threshold) continue;
			q.push(make_pair(yy, xx));
			flags.at<uchar>(yy, xx) = 1;
		}
	}
	if (xmin == inf || ymin == inf || xmax == -1 || ymax == -1){return;}
	if ( xmax <= xmin || ymax <= ymin ) {return;}
	Mat resImage;
	depth.copyTo(resImage, flags);
	resImage = resImage(Range(ymin, ymax), Range(xmin, xmax));
	resImage *= 100;
	resImage.copyTo(result);
}

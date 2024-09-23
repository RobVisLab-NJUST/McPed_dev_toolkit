#include "path_visulization.h"


VisualPath::VisualPath(ros::NodeHandle* nh)
{
    this->nh = nh;
    mat1 = cv::imread("/home/kyyyyy/matlabcode/1.png");
    theta *= DEG_TO_RAD;
    tf_sub = nh->subscribe("/tf",10,&VisualPath::tfCallBack,this);
    ros::spin();
}

void VisualPath::tfCallBack(const tf2_msgs::TFMessageConstPtr tf)
{
    int x_now = x;
    int y_now = y;
    double trans_x, trans_y, trans_x_rot, trans_y_rot;
    trans_x = tf->transforms[0].transform.translation.x;
    trans_y = tf->transforms[0].transform.translation.y;
    cout<<"origin: "<<trans_x<<' '<<trans_y<<endl;
    trans_x_rot = cos(theta)*(trans_x)-sin(theta)*trans_y;
    trans_y_rot = sin(theta)*(trans_x)+cos(theta)*trans_y;
    cout<<"rotation: "<<trans_x_rot<<' '<<trans_y_rot<<endl;
    int tran_x_pixel, tran_y_pixel;
    tran_x_pixel = trans_x_rot/0.1961;
    tran_y_pixel = trans_y_rot/0.1961;
    x_now-=tran_y_pixel;
    y_now-=tran_x_pixel;
    cv::circle(mat1,cv::Point2f(x_now,y_now),1,cv::Scalar(255,0,0),1);
    cv::imshow("PointsinImage", mat1);
    cv::waitKey(1);//敲键盘关图片，别直接×
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"draw_path");
    ros::NodeHandle nh;
    VisualPath d(&nh);
}

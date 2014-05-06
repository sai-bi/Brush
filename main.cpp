/**
 * @author 
 * @version 2014/05/06
 */
#include "cv.h"
#include "fstream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

#define PI 3.1415926



Mat_<double> place_brush(const Mat& input_image, const Mat_<double>& tangent,
        const Mat_<int>& boundary, const Mat_<int>& region_id,
        double major_axis, double minor_axis, int current_region_id){ 
    // calculate the direction of each pixel
    int width = input_image.cols;
    int height = input_image.rows;


    Mat_<int> current_region_point(width * height, 2);

    Mat_<Vec2d> inner_tangent(height,width,Vec2d(0,0));
    Mat_<int> inner_tangent_number(height,width);

    int region_point_number = 0;

    for(int i = 0;i < height;i++){
        for(int j = 0;j < width;j++){
            if(region_id(i,j) == current_region_id){
                current_region_point(region_point_number,0) = j; 
                current_region_point(region_point_number,1) = i;
                region_point_number++;
            } 
            inner_tangent(i,j)[0] = 1/sqrt(2.0);
            inner_tangent(i,j)[1] = -1/sqrt(2.0);
            inner_tangent_number(i,j) = 0;
        }
    }

    // calculate the tangent of each pixel
    vector<vector<double> > reach(width*height);
    vector<vector<Point2d> > reach_tangent(width * height);

    for(int i = 0;i < boundary.rows;i++){
        double x = boundary(i,0);
        double y = boundary(i,1);
        // double curr_tangent = tangent(i);
        double curr_tangent_x = tangent(i,0);
        double curr_tangent_y = tangent(i,1);
        // double perpendicular = -1 / curr_tangent;
        double perpendicular_x = - curr_tangent_y;
        double perpendicular_y = curr_tangent_x;


        double new_x = x;
        double new_y = y;

        double norm_value = sqrt(curr_tangent_x * curr_tangent_x + curr_tangent_y * curr_tangent_y);
        curr_tangent_x = curr_tangent_x / norm_value;
        curr_tangent_y = curr_tangent_y / norm_value;

        inner_tangent((int)y,(int)x)[0] = curr_tangent_x; 
        inner_tangent((int)y,(int)x)[1] = curr_tangent_y;
        inner_tangent_number((int)y,(int)x) = 1;

        while(true){
            if(abs(perpendicular_x) < 1e-10){
                new_x = new_x;
                new_y = new_y + 1;

            } else{
                new_x = new_x + 1;
                new_y = new_y + perpendicular_y / perpendicular_x;
            }
            //cout<<new_x<<" "<<new_y<<endl;
            if(new_x >= width || new_y >= height || new_x < 0 || new_y < 0)
                break;

            if(region_id((int)new_y,(int)new_x) != current_region_id)
                break;


            /*
               double angle = 0;
               if(abs(curr_tangent_x) < 1e-10){
               angle = PI/2;
               }
               else{
               angle = atan(curr_tangent_y / curr_tangent_x);
               if(angle < 0){
               angle = angle +	PI;
               }
               }
               */

            inner_tangent((int)new_y,(int)new_x)[0] += curr_tangent_x;
            inner_tangent((int)new_y,(int)new_x)[1] += curr_tangent_y; 
            inner_tangent_number((int)new_y,(int)new_x) += 1;
            double dist = 1.0/sqrt(pow(new_x-x,2.0) + pow(new_y-y,2.0));
            if(dist > 1e10){
                cout<<"dist is infinite"<<endl;
            }
            reach[((int)new_y)*width + (int)new_x].push_back(dist);
            //cout<<inner_tangent_number((int)new_y,(int)new_x)<<endl;
            reach_tangent[((int)new_y)*width + (int)new_x].push_back(Point2d(curr_tangent_x,curr_tangent_y));

        }
        new_x = x;
        new_y = y;
        while(true){
            if(abs(perpendicular_x) < 1e-10){
                new_x = new_x;
                new_y = new_y - 1;

            } else{
                new_x = new_x - 1;
                new_y = new_y - perpendicular_y / perpendicular_x;
            }
            if(new_x >= width || new_y >= height || new_x < 0 || new_y < 0)
                break;
            if(region_id((int)new_y,(int)new_x) != current_region_id)
                break;

            /*
               double angle = 0;
               if(abs(curr_tangent_x) < 1e-10){
               angle = PI/2;
               }
               else{
               angle = atan(curr_tangent_y / curr_tangent_x);
               if(angle < 0){
               angle = angle +	PI;
               }
               }
               */
            inner_tangent((int)new_y,(int)new_x)[0] += curr_tangent_x;
            inner_tangent((int)new_y,(int)new_x)[1] += curr_tangent_y; 
            inner_tangent_number((int)new_y,(int)new_x) += 1;
            double dist = 1.0/sqrt(pow(new_x-x,2.0) + pow(new_y-y,2.0));
            if(dist > 1e10){
                cout<<"dist is infinite"<<endl;
            }
            reach[((int)new_y)*width + (int)new_x].push_back(dist);
            reach_tangent[((int)new_y)*width + (int)new_x].push_back(Point2d(curr_tangent_x,curr_tangent_y));
            //cout<<inner_tangent_number((int)new_y,(int)new_x)<<endl;
        }
    } 

    for(int i = 0;i < height;i++){
        for(int j = 0;j < width;j++){
            if(inner_tangent_number(i,j) == 0){
                inner_tangent(i,j)[0] = 1/sqrt(2.0);
                inner_tangent(i,j)[1] = -1/sqrt(2.0);
                continue;
            }
            if(inner_tangent_number(i,j) == 1){
                //cout<<inner_tangent(i,j)[0]<<" "<<inner_tangent(i,j)[1]<<endl;
                continue;
            }
            //double angle = inner_tangent(i,j)[0] / inner_tangent_number(i,j);
            //double angle = 0;
            //inner_tangent(i,j)[0] /= inner_tangent_number(i,j);
            //inner_tangent(i,j)[1] /= inner_tangent_number(i,j);

            double sum = 0;
            for(int k = 0;k < reach[i*width+j].size();k++){
                sum = sum + reach[i*width+j][k];
            }
            /*
               if(abs(sum) < 1e-10){
               cout<<"sum is 0"<<endl;
               }
               else if(sum > 1e10){
               cout<<"sum is infinite"<<endl;
               }
               */
            double temp_x = 0;
            double temp_y = 0;
            for(int k = 0;k < reach[i*width+j].size();k++){
                //= angle + reach[i*width+j][k] / sum * inner_tangent(i,j)[0];
                temp_x = temp_x + reach_tangent[i*width+j][k].x * reach[i*width + j][k] / sum;
                temp_y = temp_y + reach_tangent[i*width+j][k].y * reach[i*width + j][k] / sum;
            }

            //double x = inner_tangent(i,j)[0];
            //double y = inner_tangent(i,j)[1];
            double temp = sqrt(temp_x*temp_x + temp_y*temp_y);

            inner_tangent(i,j)[0] = temp_x / temp;
            inner_tangent(i,j)[1] = temp_y / temp;

            //cout<<inner_tangent(i,j)[0]<<" "<<inner_tangent(i,j)[1]<<endl;
        }
    }

    // random seed

    int seed_number = 1.3 * region_point_number / (PI * major_axis * minor_axis);
    Mat test_image = Mat::zeros(height,width,CV_8UC3);
    Mat_<int> seed_point(seed_number,2);
    RNG random_generator(getTickCount());
    for(int i = 0;i < seed_number;i++){
        int index = random_generator.uniform(0,region_point_number);
        int temp_x = current_region_point(index,0);
        int temp_y = current_region_point(index,1);

        bool inside_ellipse = false;
        for(int j = 0;j < i;j++){
            int seed_x = seed_point(j,0);
            int seed_y = seed_point(j,1);
            double seed_tangent_x = inner_tangent(seed_y,seed_x)[0];
            double seed_tangent_y = inner_tangent(seed_y,seed_x)[1]; 
            double dist = sqrt(pow(temp_x - seed_x,2.0) + pow(temp_y - seed_y,2.0));

            Point2d temp1(temp_x - seed_x,temp_y - seed_y);
            Point2d temp2(seed_tangent_x,seed_tangent_y); 
            double cos_theta = (temp1.x * temp2.x + temp1.y * temp2.y) / (norm(temp1) * norm(temp2)); 
            double x = dist * cos_theta;
            double y = dist * sqrt(1 - cos_theta * cos_theta);
            dist = sqrt(pow(x/major_axis,2.0) + pow(y/minor_axis,2.0)); 

            if(dist < 1){
                inside_ellipse = true;
                break; 
            }
        }

        if(inside_ellipse == true){
            i--;
            // break;
            continue;
        }
        double curr_tangent_x = inner_tangent((int)temp_y,(int)temp_x)[0];
        double curr_tangent_y = inner_tangent((int)temp_y,(int)temp_x)[1];
        double angle = atan(curr_tangent_y / curr_tangent_x);
        /*
           if(angle < 0)
           angle = angle + PI;
           angle = angle * 180 / PI;
           cout<<curr_tangent_x<<"  "<<curr_tangent_y<<endl;
           cout<<angle<<endl;
           */

        //ellipse(test_image,Point2d(temp_x,temp_y),Size(major_axis,minor_axis),angle,0,360,Scalar(0,0,255));
        //circle(test_image, Point2d(temp_x,temp_y),3,Scalar(255,0,0));

        //imshow("seed",test_image);
        //waitKey(5);

        seed_point(i,0) = temp_x;
        seed_point(i,1) = temp_y;    
    }

    // K-means clustering
    vector<vector<int> > cluster_index(seed_number);
    double threshold = 5;
    double color_weight = 0.01;

    vector<Vec3b> color;
    for(int i = 0;i < seed_number;i++){
        Vec3b temp;
        temp[0] = random_generator.uniform(0,256);
        temp[1] = random_generator.uniform(0,256);
        temp[2] = random_generator.uniform(0,256);
        color.push_back(temp);
    }


    Mat temp_image;
    while(true){
        temp_image = Mat::zeros(height,width,CV_8UC3);
        cluster_index.clear();
        cluster_index.resize(seed_number);
        for(int i = 0;i < region_point_number;i++){
            double temp_x = current_region_point(i,0);
            double temp_y = current_region_point(i,1); 
            int min_index = 0;
            double min_dist = 1e10;
            Vec3b curr_point_rgb = input_image.at<Vec3b>((int)temp_y,(int)temp_x);
            for(int j = 0;j < seed_number;j++){
                int seed_x = seed_point(j,0);
                int seed_y = seed_point(j,1);
                double seed_tangent_x = inner_tangent(seed_y,seed_x)[0];
                double seed_tangent_y = inner_tangent(seed_y,seed_x)[1]; 
                double dist = sqrt(pow(temp_x - seed_x,2.0) + pow(temp_y - seed_y,2.0));

                Point2d temp1(temp_x - seed_x,temp_y - seed_y);
                Point2d temp2(seed_tangent_x,seed_tangent_y); 
                if(abs(norm(temp1)) < 1e-10){
                    min_index = j;
                    break;
                }
                double cos_theta = (temp1.x * temp2.x + temp1.y * temp2.y) / (norm(temp1) * norm(temp2)); 
                double x = dist * cos_theta;
                double y = dist * sqrt(1 - cos_theta * cos_theta);
                dist = sqrt(pow(x/major_axis,2.0) + pow(y/minor_axis,2.0)); 

                Vec3b seed_rgb = input_image.at<Vec3b>(seed_y,seed_x);

                double color_dist = norm(curr_point_rgb - seed_rgb); 

                dist = dist + color_dist * color_weight;
                if(dist < min_dist){
                    min_dist = dist;
                    min_index = j;
                } 
            }
            temp_image.at<Vec3b>((int)temp_y,(int)temp_x) = color[min_index];
            cluster_index[min_index].push_back(i);
        }


        // calculate new seed point
        Mat_<int> old_seed_point = seed_point.clone();
        for(int i = 0;i < cluster_index.size();i++){
            double x = 0;
            double y = 0;
            for(int j = 0;j < cluster_index[i].size();j++){
                double temp_x = current_region_point(cluster_index[i][j],0);
                double temp_y = current_region_point(cluster_index[i][j],1);
                x = x + temp_x;
                y = y + temp_y;
            }
            if(cluster_index[i].size() == 0){
                cout<<"cluster is 0"<<endl;
            }
            //cout<<cluster_index[i].size()<<endl;
            if(cluster_index[i].size() != 0){
                x = x / cluster_index[i].size();
                y = y / cluster_index[i].size();
            }

            seed_point(i,0) = (int)x;
            seed_point(i,1) = (int)y;
        } 

        double difference = norm(seed_point - old_seed_point);

        for(int i = 0;i < seed_number;i++){
            double x = seed_point(i,0);
            double y = seed_point(i,1);

            if(x >= width || y >= height || x < 0 || y < 0){
                cout<<"point exceed"<<endl;
                //break;
            }


            double tangen_x = inner_tangent((int)y,(int)x)[0];
            double tangen_y = inner_tangent((int)y,(int)x)[1];
            /*
               double angle = atan(tangen_y / tangen_x);
            //cout<<"wow"<< angle<<endl;
            if(angle < 0)
            angle = angle + PI;
            angle = angle * 180 / PI;
            cout<<"hello"<<angle<<endl;
            */
            //cout<<tangen_x<<" "<<tangen_y<<" hello"<<endl;
            //ellipse(temp_image,Point2d(x,y),Size(major_axis,minor_axis),angle,0,360,Scalar(255,255,255),3);
            circle(temp_image, Point2d(x,y),5,Scalar(255,255,255));
            double end_x = x;
            double end_y = y;
            if(abs(tangen_x)<1e-10){
                end_x = x;
                end_y = y + 10;
            }
            else{
                end_x = x + 10 * tangen_x / sqrt(tangen_x*tangen_x + tangen_y*tangen_y);
                end_y = y +  10 * tangen_y / sqrt(tangen_x*tangen_x + tangen_y*tangen_y);
            }
            line(temp_image,Point2d(x,y),Point2d(end_x,end_y),Scalar(255,255,255),3);

            /*
               for(int j = 0;j < reach[width *(int)y + (int)x].size();j++){
               double temp_x = reach[width *(int)y + (int)x][j].x;
               double temp_y = reach[width *(int)y + (int)x][j].y;
               double tangen_x = inner_tangent((int)temp_y,(int)temp_x)[0];
               double tangen_y = inner_tangent((int)temp_y,(int)temp_x)[1];
               if(abs(tangen_x) < 1e-10){
               cout<<"hello 0"<<endl;
               }

               double angle = atan(tangen_y / tangen_x);
            //cout<<"wow"<< angle<<endl;
            if(angle < 0)
            angle = angle + PI;
            angle = angle * 180 / PI;
            cout<<angle<<" ";
            }
            */




        }


        imshow("result",temp_image);
        waitKey(5);
        cout<<difference<<endl;

        if(difference < threshold){
            break;
        }
    }


    Mat_<double> result(seed_number,4);

    for(int i = 0;i < seed_number;i++){
        result(i,0) = seed_point(i,0);
        result(i,1) = seed_point(i,1);
        result(i,2) = inner_tangent((int)seed_point(i,1),(int)seed_point(i,0))[0]; 
        result(i,3) = inner_tangent((int)seed_point(i,1),(int)seed_point(i,0))[1]; 
    }

    return result;

}
void my_show(const Mat_<int>& boundary,int num){
    Mat src = Mat::zeros( Size(800,800), CV_8UC1 );
    for(int i = 0;i < num-1;i++){
        double x_cor = boundary(i,0);
        double y_cor = boundary(i,1);
        double end_x = boundary(i+1,0);
        double end_y = boundary(i+1,1);
        cout<<end_x<<" "<<end_y<<endl;
        line(src,Point2d(x_cor,y_cor),Point2d(end_x,end_y),Scalar(255),3,8); 
    }
    imshow("src",src);
    waitKey(0);
}

void calculate_tangent(const Mat& input_image, 
        const Mat_<int>& boundary, const Mat_<int>& region_id,int current_region_id,
        Mat_<Point2d>& tangent){

    int width = input_image.cols;
    int height = input_image.rows;

    Mat_<int> visited(height,width,0);
    Mat_<int> current_boundary;

    current_boundary = boundary.clone();
    int current_boundary_size = current_boundary.rows;

    for(int i = 0;i < boundary.rows;i++){
        int x = boundary(i,0);
        int y = boundary(i,1);
        visited(y,x) = 1;
    }


    while(true){
        // find new boundary after thinning
        int new_boundary_size = 0;	
        Mat_<int> new_boundary(width*height,2);
        for(int i = 0;i < current_boundary_size;i++){
            int x = current_boundary(i,0);
            int y = current_boundary(i,1);


            for(int j = -1;j < 2;j++){
                for(int k = -1;k < 2;k++){
                    if(j * k != 0 || (j==0 && k==0)){
                        continue;
                    }


                    int new_x = x + j;
                    int new_y = y + k;
                    if(new_x < 0 || new_y < 0 || new_x >= width || new_y >= height){
                        continue;
                    }

                    if(region_id(new_y,new_x) != current_region_id){
                        continue;
                    }

                    if(visited(new_y,new_x) == 1){
                        continue;
                    }

                    new_boundary(new_boundary_size,0) = new_x;
                    new_boundary(new_boundary_size,1) = new_y;
                    new_boundary_size = new_boundary_size + 1;
                    visited(new_y,new_x) = 1;
                    tangent(new_y,new_x) = tangent(y,x);
                }
            }
        }
        // my_show(new_boundary,new_boundary_size);
        current_boundary = new_boundary.clone();
        current_boundary_size = new_boundary_size;
        // if no boundary points found, exit
        if(new_boundary_size == 0){
            break;
        }
    }
}


void cal_tangent(const Mat_<int>& boundary, Mat_<Point2d> & tangent){
    // tangent = Mat_<double>(boundary.size(),2);
    int tangent_number = 6;
    int boundary_point_number = boundary.rows;
    for(int j = 0;j < boundary.rows;j++){
        double tangent_x = 0;
        double tangent_y = 0;
        for(int k = 0;k < tangent_number;k++){
            int index1 = (j + k) % boundary_point_number;
            int index2 = (j + k + 1) % boundary_point_number;

            double x = boundary(index2,0) - boundary(index1,0);
            double y = boundary(index2,1) - boundary(index1,1);

            // if(abs(x)<1e-10){
            //angle = angle + PI/2;
            // tangent_x += 0;
            // tangent_y += 1;
            // } else{
            //angle = angle + atan(y/x);
            double temp = sqrt(x*x + y*y);
            tangent_x += x / temp;
            tangent_y += y / temp;


            index2 = (j - k + boundary_point_number) % boundary_point_number;
            index1 = (j - k - 1 + boundary_point_number) % boundary_point_number;
            x = boundary(index2,0) - boundary(index1,0);
            y = boundary(index2,1) - boundary(index1,1);

            // if(abs(x)<1e-10){
            // tangent_x += 0;
            // tangent_y += 1;
            // } else{
            temp = sqrt(x*x + y*y);
            tangent_x += x / temp;
            tangent_y += y / temp;

        }

        double temp = sqrt(tangent_x*tangent_x + tangent_y*tangent_y);
        tangent_x = tangent_x /temp;
        tangent_y = tangent_y / temp;

        // tangent(j,0) = tangent_x;
        // tangent(j,1) = tangent_y; 
        tangent(boundary(j,1),boundary(j,0)) = Point2d(tangent_x,tangent_y);

        // cout<<tangent_x<<" "<<tangent_y<<endl;
        }
        }



        void smooth_tangent(Mat_<Point2d>& tangent, const Mat_<int> region_id,int current_region_id){
            int width = tangent.cols;
            int height = tangent.rows;

            Mat_<double> tangent_angle(height,width,double(0));

            for(int i = 0;i < width;i++){
                for(int j = 0;j < height;j++){
                    if(region_id(j,i) != current_region_id){
                        continue;
                    }
                    Point2d temp = tangent(j,i);
                    // double angle = atan2(temp.y,temp.x); 
                    double angle = 0;
                    if(abs(temp.x) < 1e-10){
                        if(temp.y > 0){
                            angle = PI/2;
                        }
                        else{
                            angle = -PI/2;
                        }
                    }
                    else{
                        angle = atan(temp.y / temp.x);
                    }
                    tangent_angle(j,i) = angle * 2;
                }
            } 

            int iter_num = 0; 
            while(iter_num< 100){
                Mat_<double> new_tangent_angle = tangent_angle.clone();
                // Mat_<Point2d> new_tangent(height,width,Point2d(0,0));
                for(int i = 0;i < width;i++){
                    for(int j = 0;j < height;j++){
                        if(region_id(j,i) != current_region_id){
                            continue;
                        } 

                        int neighbour_num = 3; 
                        double delta_theta = 0;
                        for(int p = -neighbour_num;p < neighbour_num + 1;p++){
                            for(int k = -neighbour_num;k < neighbour_num+1;k++){
                                int x = i + p;
                                int y = j + k;
                                if(x < 0 || y < 0 || x >= width || y >= height){
                                    continue;
                                }
                                if(region_id(y,x) != current_region_id){
                                    continue;
                                }

                                delta_theta = delta_theta + sin(tangent_angle(y,x) - tangent_angle(j,i));         
                            } 
                        }

                        new_tangent_angle(j,i) += delta_theta; 
                    } 
                }

                tangent_angle = new_tangent_angle.clone();
                iter_num++;
            }

            for(int i = 0; i < width;i++){
                for(int j = 0;j < height;j++){
                    if(region_id(j,i) != current_region_id){
                        continue;
                    }
                    tangent(j,i).x = cos(tangent_angle(j,i) / 2);
                    tangent(j,i).y = sin(tangent_angle(j,i) / 2); 
                }
            }

        }


        int main(){
            ifstream fin;
            fin.open("./point.txt");
            int id;
            int point_number;
            Mat image = imread("./test.jpg");
            int width = image.cols;
            int height = image.rows;

            int count = 0;

            while(fin>>id>>point_number){
                cout<<point_number<<endl;
                vector<Point2f> point_list; 
                for(int i = 0;i < point_number;i++){
                    int x = 0;
                    int y = 0;
                    fin>>x>>y;
                    point_list.push_back(Point2f(x,y));        
                }

                if(count == 0){
                    count++;
                    continue;
                } 


                vector<vector<Point> > contours;
                vector<Vec4i> hierarchy;
                Mat src = Mat::zeros( Size(width,height), CV_8UC1 );
                for( int j = 0; j < point_list.size(); j++ ){ 
                    line( src, point_list[j],  point_list[(j+1)%point_list.size()], Scalar( 255 ), 3, 8 ); 
                } 


                findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

                Mat_<int> region_id(height,width,0); 

                for(int i = 0;i < height;i++){
                    for(int j = 0;j < width;j++){
                        double inside = pointPolygonTest(contours[0], Point2f(j,i), false);
                        if(inside >= 0){
                            region_id(i,j) = 1;
                        }
                        else{
                            region_id(i,j) = 0;
                        } 
                    }
                }
                Mat_<int> boundary(point_list.size(),2);

                for(int i = 0;i < point_list.size();i++){
                    boundary(i,0) = point_list[i].x;
                    boundary(i,1) = point_list[i].y;  
                }


                Mat_<Point2d> tangent(height,width,Point2d(0,0));

                cal_tangent(boundary,tangent);

                /*
                   for(int i = 0;i < boundary.rows;i = i + 10){
                   double x_cor = boundary(i,0);
                   double y_cor = boundary(i,1);
                   double x = tangent(y_cor,x_cor).x;
                   double y = tangent(y_cor,x_cor).y;
                   double end_x = x_cor + 10 * x / sqrt(x*x+y*y);
                   double end_y = y_cor + 10 * y / sqrt(x*x+y*y);

                   cout<<x<<" "<<y<<endl;
                   line(src,Point2d(x_cor,y_cor),Point2d(end_x,end_y),Scalar(255),3,8); 
                   imshow("src",src);
                   waitKey(5);
                   }
                   */


                calculate_tangent(image,boundary,region_id,1,tangent);
                smooth_tangent(tangent,region_id,1);

                for(int i = 0;i < width;i=i+20){
                    for(int j = 0;j < height;j=j+20){
                        if(region_id(j,i) == 1){
                            double x = tangent(j,i).x;
                            double y = tangent(j,i).y;
                            double end_x = i + 10 * x / sqrt(x*x+y*y);
                            double end_y = j + 10 * y / sqrt(x*x+y*y);
                            cout<<i<<" "<<j<<" "<<" "<<x<<y<<endl;
                            line(src,Point2d(i,j),Point2d(end_x,end_y),Scalar(255),1,8);

                            Point2d pStart(i,j);
                            Point2d pEnd(end_x,end_y); 
                            Point arrow;
                            int len = 4;
                            int alpha = 15;
                            double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));  
                            arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);     
                            arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);  
                            line(src, pEnd, arrow, Scalar(255), 1, 8);
                            arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);     
                            arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);    
                            line(src, pEnd, arrow, Scalar(255), 1, 8);


                            // imshow("src",src); 
                            // waitKey(0);
                        } 
                    }
                }    
                imshow("src",src); 
                waitKey(0);
            } 

            return 0;

        }



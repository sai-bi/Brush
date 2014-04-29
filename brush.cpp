/**
 * @author 
 * @version 2014/04/27
 */

#include <iostream>
using namespace std;

Mat_<double> place_brush(const Mat& input_image, const Mat_<double>& tangent,
        const Mat_<int>& boundary, const Mat_<int>& region_id,
        double major_axis, double minor_axis, int current_region_id){ 
    // calculate the direction of each pixel
    int width = input_image.cols;
    int height = input_image.rows;

    Mat_<int> current_region_point(width * height, 2);

    Mat_<double> inner_tangent = Mat_<double>::zeros(height,width,2);
    Mat_<int> inner_tangent_number = Mat_<int>::zeros(height,width,2);

    int region_point_number = 0;

    for(int i = 0;i < height;i++){
        for(int j = 0;j < width;j++){
            if(region_id(i,j) == current_region_id){
                current_region_point(region_point_number,0) = j; 
                current_region_point(region_point_number,1) = i;
                region_point_number++;
            } 
        }
    }

    // calculate the tangent of each pixel
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

        inner_tangent((int)y,(int)x,0) = curr_tangent_x; 
        inner_tangent((int)y,(int)x,1) = curr_tangent_y;

        while(true){
            if(new_x != 0){
                new_x = new_x + 1;
                new_y = new_y + perpendicular_y / perpendicular_x;
            } else{
                new_x = new_x;
                new_y = new_y + 1;
            }

            if(new_x >= width || new_y >= height || new_x < 0 || new_y < 0)
                break;

            if(region_id((int)new_y,(int)new_x) != current_region_id)
                break;

            inner_tangent((int)new_y,(int)new_x,0) = curr_tangent_x;
            inner_tangent((int)new_y,(int)new_x,1) = curr_tangent_y; 
            inner_tangent_number((int)new_y,(int)new_x) += 1;
        }
        while(true){
            if(new_x != 0){
                new_x = new_x - 1;
                new_y = new_y - perpendicular_y / perpendicular_x;
            } else{
                new_x = new_x;
                new_y = new_y - 1;
            }
            if(new_x >= width || new_y >= height || new_x < 0 || new_y < 0)
                break;
            if(region_id((int)new_y,(int)new_x) != current_region_id)
                break;
            inner_tangent((int)new_y,(int)new_x,0) = curr_tangent_x;
            inner_tangent((int)new_y,(int)new_x,1) = curr_tangent_y; 
            inner_tangent_number((int)new_y,(int)new_x) += 1;
        }
    } 

    for(int i = 0;i < height;i++){
        for(int j = 0;j < width;j++){
            if(inner_tangent_number(i,j) == 0) 
                continue;
            inner_tangent(i,j,0) /= inner_tangent_number(i,j);
            inner_tangent(i,j,1) /= inner_tangent_number(i,j);
        }
    }

    // random seed
    double PI = 3.1415926;
    int seed_number = 1.5 * region_point_number / (PI * major_axis * minor_axis);

    Mat_<int> seed_point(seed_number,2);
    RNG random_generator(getTickCount());
    for(int i = 0;i < seed_number;i++){
        int index = random_generator.uniform(0,region_point_number);
        int temp_x = current_region_point(index,0);
        int temp_y = current_region_point(index,1);

        bool inside_ellipse = false;
        for(int j = 0;j < i;j++){
            int seed_x = seed_point(i,0);
            int seed_y = seed_point(i,1);
            // double dist = pow(temp_x - seed_x,2.0) + pow(temp_y - seed_y,2.0);
            // double seed_tangent = inner_tangent(seed_y,seed_x); 
            // double x = dist * 1.0 / sqrt(seed_tangent * seed_tangent + 1);
            // double y = dist * seed_tangent / sqrt(seed_tangent * seed_tangent + 1);
            double seed_tangent_x = inner_tangent(seed_y,seed_x,0);
            double seed_tangent_y = inner_tangent(seed_y,seed_x,1); 
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

        seed_point(i,0) = temp_x;
        seed_point(i,1) = temp_y;    
    }

    // K-means clustering
    vector<vector<int> > cluster_index(seed_number);
    double threshold = 5;
    double color_weight = 0.01;

    while(true){
        cluster_index.clear();
        cluster_index.resize(seed_number);
        for(int i = 0;i < region_point_number;i++){
            double temp_x = current_region_point(i,0);
            double temp_y = current_region_point(i,1); 
            double min_index = 0;
            double min_dist = 1e10;
            Vec3b curr_point_rgb = input_image.at<Vec3b>((int)temp_y,(int)temp_x);
            for(int j = 0;j < seed_number;j++){
                int seed_x = seed_point(j,0);
                int seed_y = seed_point(j,1);
                double seed_tangent_x = inner_tangent(seed_y,seed_x,0);
                double seed_tangent_y = inner_tangent(seed_y,seed_x,1); 
                double dist = sqrt(pow(temp_x - seed_x,2.0) + pow(temp_y - seed_y,2.0));

                Point2d temp1(temp_x - seed_x,temp_y - seed_y);
                Point2d temp2(seed_tangent_x,seed_tangent_y); 
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
            x = x + seed_point(i,0);
            y = y + seed_point(i,1);
            x = x / (cluster_index[i].size() + 1);
            y = y / (cluster_index[j].size() + 1);

            seed_point(i,0) = (int)x;
            seed_point(i,1) = (int)y;
        } 

        double difference = norm(seed_point - old_seed_point);

        if(difference < threshold){
            break;
        }
    }

    Mat_<double> result(seed_number,4);

    for(int i = 0;i < seed_number;i++){
        result(i,0) = seed_point(i,0);
        result(i,1) = seed_point(i,1);
        result(i,2) = inner_tangent((int)seed_point(i,1),(int)seed_point(i,0),0); 
        result(i,3) = inner_tangent((int)seed_point(i,1),(int)seed_point(i,0),1); 
    }

    return result;

}




void calculate_tangent(const Mat& input_image, const Mat_<double>& tangent,
	const Mat_<int>& boundary, const Mat_<int>& region_id,int current_region_id,
	Mat_<double>& tangent){
	
	int width = input_image.cols;
	int height = input_image.rows;

	Mat_<int> visited(height,width);
	Mat_<int> current_boundary(width*height,2);
	
	current_boundary = boundary.clone();
	int current_boundary_size = current_boundary.rows();


	while(true){
		// find new boundary after thinning
		for(int i = 0;i < current_boundary_size;i++){
			int x = current_boundary(i,0);
			int y = current_boundary(i,1);
			Mat_<int> new_boundary(width*height,2);
			int new_boundary_size = 0;	

			for(int j = -1;j < 2;j++){
				for(int k = -1;k < 2;k++){
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

		// if no boundary points found, exit
		if(new_boundary_size == 0){
			break;
		}
	}
}

void smooth_tangent(Mat_<double> tangent){
						
}


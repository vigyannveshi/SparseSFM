/**
 * @file visualizer.cpp
 * @brief Implementation of Pangolin-based 3D viewer.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "visualizer.hpp"
#include <algorithm>


// function to get colors for landmark points
std::vector<cv::Vec3b> get_landmark_colors(
    const std::vector<Landmark>& landmarks,
    const std::vector<Frame>& frames
){
    // vector to store color of landmark keypoints
    std::vector<cv::Vec3b> colors;

    // vectors for image caching
    std::list<int> lru_order; // least recently used list
    std::unordered_map<int, cv::Mat> image_cache;
    
    for (const Landmark& lm: landmarks){
        // get the first observation where the landmark is observed
        const Observation& obs = lm.get_observations()[0];
        
        // get the frame having the observation
        const Frame& frame = frames[obs.frame_idx];

        // cache - logic for image caching
        int fid = frame.get_id();

        // is image-already cached?
        if (image_cache.count(fid)){
            // hit - move to recent
            lru_order.remove(fid);
            lru_order.push_front(fid);
        }
        else{
            // miss - load and insert
            if (static_cast<int>(image_cache.size()) >= PANGOLIN_IMG_BUFFER_SIZE){
                // evict least recently used
                int evict = lru_order.back();
                lru_order.pop_back();
                image_cache.erase(evict); // remove from cache
            }
            image_cache[fid] = cv::imread(frame.get_image_path());
            lru_order.push_front(fid);
        }

        // get the keypoint of the observation
        cv::KeyPoint kp = frame.get_keypoints()[obs.keypoint_idx];

        // get the pixel coordinates of the keypoint
        int x = static_cast<int>(kp.pt.x);
        int y = static_cast<int>(kp.pt.y);

        // row is y, col is x. OpenCV is row-major (similar convention to Rafael C. Gonzalez & Richard E. Woods)
        colors.push_back(image_cache[fid].at<cv::Vec3b>(y,x));
    }
    return colors;
}


// function to visualize point-clouds using landmarks
void visualize_landmarks(const std::vector<Landmark>& landmarks, 
                         const std::vector<Frame>& frames,
                         const std::vector<Pose>& poses,
                         const std::string& title
                        ){
    // estimate landmark point cloud point colors
    std::vector<cv::Vec3b> colors = get_landmark_colors(landmarks, frames);

    // Step 1. Create Pangolin window
    pangolin::CreateWindowAndBind(
        title,
        PANGOLIN_WINDOW_WIDTH,
        PANGOLIN_WINDOW_HEIGHT
    );

    // tells OpenGL to maintain a depth buffer — for every pixel, it tracks the depth of what's drawn there and only overwrites it if the new thing is closer.
    glEnable(GL_DEPTH_TEST);

    // ensure that background color is white
    glClearColor(PANGOLIN_BACKGROUND_COLOR.R,
                 PANGOLIN_BACKGROUND_COLOR.G,
                 PANGOLIN_BACKGROUND_COLOR.B,
                 PANGOLIN_BACKGROUND_COLOR.alpha);
    
    // Step 2. define the render state (virtual camera looking into the scene)
    pangolin::OpenGlRenderState s_cam(
        // define the viewer lens
        pangolin::ProjectionMatrix(
            PANGOLIN_WINDOW_WIDTH,
            PANGOLIN_WINDOW_HEIGHT,
            PANGOLIN_VIEWER_FX,
            PANGOLIN_VIEWER_FY,
            PANGOLIN_VIEWER_CX,
            PANGOLIN_VIEWER_CY,
            PANGOLIN_VIEWER_NEAR_CLIP,
            PANGOLIN_VIEWER_FAR_CLIP
        ),

        // defines the position of the viewer
        pangolin::ModelViewLookAt(
            // location of viewer's eye
            PANGOLIN_VIEWER_EX, 
            PANGOLIN_VIEWER_EY,
            PANGOLIN_VIEWER_EZ,
            // location at which viewer is looking at
            PANGOLIN_VIEWER_LX, 
            PANGOLIN_VIEWER_LY,
            PANGOLIN_VIEWER_LZ,
            pangolin::AxisNegY
        )
    );

    // Step 3. create the display and attach the render state
    pangolin::View& d_cam = pangolin::CreateDisplay();

    // 0.0 to 1.0 normalized coordinates defining how much of the window this view occupies. All four at 0 and 1 means full window.
    d_cam.SetBounds(0.0, 1.0, 0.0, 1.0, -static_cast<float>(PANGOLIN_WINDOW_WIDTH)/static_cast<float>(PANGOLIN_WINDOW_HEIGHT));
    
    // attaches mouse interaction to this view, tied to s_cam. This is what lets you rotate/pan/zoom with the mouse. Handler3D modifies s_cam's ModelViewLookAt internally as you drag.
    d_cam.SetHandler(new pangolin::Handler3D(s_cam));

    //  Step 4. render the loop
    while (!pangolin::ShouldQuit()){
        // at the start of each frame resets that buffer, otherwise color/depth values from the previous frame bleed into the current one.
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // binds the render state so mouse interaction (rotate, zoom) works automatically.
        d_cam.Activate(s_cam);

        // Step 5. Drawing points
        glPointSize(PANGOLIN_POINT_CLOUD_POINT_SIZE); // setting point size

        // draw camera frustrums
        if (!poses.empty())
        {
            for (const Pose& pose: poses){
                draw_camera_frustum(pose);
            }
            if (poses.size()>1){
                for (int i = 0; i<poses.size()-1; i++){
                    std::pair<Pose,Pose> pair = std::make_pair(poses[i],poses[i+1]);
                    trace_path(pair, PANGOLIN_CAMERA_PATH_COLOR, PANGOLIN_CAMERA_PATH_LINE_WIDTH);
                }
            }
        }

        // Everything between glBegin and glEnd is one batch submitted together.
        // GL_POINTS tells OpenGL each glVertex call is an independent point. Other primitives include GL_LINES, GL_TRIANGLES etc.
        glBegin(GL_POINTS); 
            for (int i = 0; i < landmarks.size(); i++){
                // opencv uses BGR convention
                glColor3ub(colors[i][2],colors[i][1], colors[i][0]);
                cv::Point3d pos = landmarks[i].get_position();
                
                // drawing the point
                glVertex3d(pos.x, pos.y, pos.z);
            }
        glEnd();

        // finish frame
        pangolin::FinishFrame();
    } 
}


// function to visualize camera pose (single)
void draw_camera_frustum(const Pose& pose){
    // transform each frustum point into reference frame using pose
    cv::Point3d O  = camera_to_ref_frame(camera_frustum.O, pose);
    cv::Point3d p1 = camera_to_ref_frame(camera_frustum.p1, pose);
    cv::Point3d p2 = camera_to_ref_frame(camera_frustum.p2, pose);
    cv::Point3d p3 = camera_to_ref_frame(camera_frustum.p3, pose);
    cv::Point3d p4 = camera_to_ref_frame(camera_frustum.p4, pose);

    // draw frustrum lines
    glColor3f(PANGOLIN_CAMERA_FRUSTUM_COLOR.R,
              PANGOLIN_CAMERA_FRUSTUM_COLOR.G,
              PANGOLIN_CAMERA_FRUSTUM_COLOR.B);
    glLineWidth(PANGOLIN_CAMERA_FRUSTUM_LINE_WIDTH);
    glBegin(GL_LINES);
        // pyramid edges (O to each corner)
        glVertex3d(O.x, O.y, O.z);  glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(O.x, O.y, O.z);  glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(O.x, O.y, O.z);  glVertex3d(p3.x, p3.y, p3.z);
        glVertex3d(O.x, O.y, O.z);  glVertex3d(p4.x, p4.y, p4.z);
        
        // base rectangle
        glVertex3d(p1.x, p1.y, p1.z);  glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p2.x, p2.y, p2.z);  glVertex3d(p3.x, p3.y, p3.z);
        glVertex3d(p3.x, p3.y, p3.z);  glVertex3d(p4.x, p4.y, p4.z);
        glVertex3d(p4.x, p4.y, p4.z);  glVertex3d(p1.x, p1.y, p1.z);
    glEnd();
}

// function to trace poses (draws line between two given poses)
void trace_path(const std::pair<Pose,Pose>& pose_pair, const Color& color, const float& linewidth){
    // getting points to trace path given pose
    cv::Point3d p1 = camera_to_ref_frame({0,0,0}, pose_pair.first);
    cv::Point3d p2 = camera_to_ref_frame({0,0,0}, pose_pair.second);

    // tracing path
    glLineWidth(linewidth);
    glColor3f(color.R, color.G, color.B);
    glBegin(GL_LINE_STRIP);
        glVertex3d(p1.x, p1.y, p1.z); glVertex3d(p2.x, p2.y, p2.z); 
    glEnd();
}
#include "image.h"

const uint32_t kInvalidId = std::numeric_limits<uint32_t>::max();

Image::Image()
    :image_id_(kInvalidId),
     name_(""),
     camera_id_(kInvalidId),
     registerd_(false),
     num_points3D_(0),
     num_observations_(0),
     num_correspondences_(0),
     qvec_(1.0, 0.0, 0.0, 0.0),
     tvec_(0.0, 0.0, 0.0){}
    


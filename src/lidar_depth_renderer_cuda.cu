#include "lidar_depth_renderer/lidar_depth_renderer_cuda.h"
#include <cuda.h>
#include <cuda_runtime.h>

/* -------------------- CUDA Util Functions ------------------------*/
#define CHECK_GPU(ans) \
  { gpu_assert((ans), __FILE__, __LINE__); }
#define CHECK_GPU_SAFE(ans) \
  { gpu_assert((ans), __FILE__, __LINE__, false); }
inline void gpu_assert(cudaError_t code, const char* file, int line,
                       bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort) {
      exit(code);
    }
  }
}

void *cuda_malloc(const size_t size) {
  void *cloud_ptr;
  CHECK_GPU(cudaMalloc(&cloud_ptr, size));
  return cloud_ptr;
}

void cuda_memcpy_to_dev(void *dst, const void *src, const size_t size) {
  CHECK_GPU(cudaMemcpy(dst, src, size, cudaMemcpyHostToDevice));
}

void cuda_free(void *dst) {
  CHECK_GPU(cudaFree(dst));
}

/* -------------------- CUDA Structures Functions ------------------------*/
// make device available camera info
struct CudaCameraInfo {
  int width;
  int height;

  float fx;
  float fy;
  float cx;
  float cy;

  CudaCameraInfo(const sensor_msgs::CameraInfo &info)
      : width(info.width),
        height(info.height),
        fx(info.K[0]),
        fy(info.K[4]),
        cx(info.K[2]),
        cy(info.K[5]) {}
}; // Struct CudaCameraInfo

// Need to have padding to correctly interpret PCL PointXYZ
struct CudaPoint {
  float x;
  float y;
  float z;
  float padding;
}; // Struct CudaPoint

// make device available transform info
struct CudaTransform {
  float rot[3][3];
  float trans[3];

  CudaTransform(const tf2::Transform &tf) {
    const auto &basis = tf.getBasis();
    const auto &origin = tf.getOrigin();
    for (int i = 0; i < 3; i++) {
      trans[i] = origin[i];
      for (int j = 0; j < 3; j++) {
        rot[i][j] = basis[i][j];
      }
    }
  }

}; // Struct CudaTransform

/* -------------------- CUDA Kernel Functions ------------------------*/
// stores result to image_scratch_buf
// image_scratch_buf should be set to all 255
__global__ void kernel_render(const CudaPoint *cloud_points,
                              const size_t num_points,
                              const CudaCameraInfo camera_info,
                              const CudaTransform to_camera_tf,
                              const int bloat_factor,
                              uint32_t *image_scratch_buf) {
  // get point index
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx > num_points) return;
  const auto point = cloud_points[idx];

  // transform point
  const auto &trans = to_camera_tf.trans;
  const auto &rot = to_camera_tf.rot;

  const float x = rot[0][0] * point.x +
                  rot[0][1] * point.y +
                  rot[0][2] * point.z + trans[0];
  const float y = rot[1][0] * point.x +
                  rot[1][1] * point.y +
                  rot[1][2] * point.z + trans[1];
  const float z = rot[2][0] * point.x +
                  rot[2][1] * point.y +
                  rot[2][2] * point.z + trans[2];
  if (z <= 0) return;
  
  // project point
  const auto fx = camera_info.fx;
  const auto fy = camera_info.fy;
  const auto cx = camera_info.cx;
  const auto cy = camera_info.cy;

  const auto u = static_cast<int>((fx * x) / z + cx);
  const auto v = static_cast<int>((fy * y) / z + cy);

  // render point
  const auto width = camera_info.width;
  const auto height = camera_info.height;

  if (u - bloat_factor >= 0 && u + bloat_factor < width &&
      v - bloat_factor >= 0 && v + bloat_factor < height) {
    auto cur_depth = static_cast<uint32_t>(z * 2.0);
    // cap to 254
    cur_depth = cur_depth > 255 ? 254 : cur_depth;
    for (int i = -bloat_factor; i <= bloat_factor; i++) {
      for (int j = -bloat_factor; j <= bloat_factor; j++) {
        // convert to 1d buffer
        const auto buf_index = (v + i) * width + u;
        // atomic min to buffer
        atomicMin(image_scratch_buf + buf_index, cur_depth);
      }
    }
  }
}

__global__ void kernel_populate_image_buf(uint32_t *image_scratch_buf,
                                          uint8_t *image_buf,
                                          const size_t num_pixels) {
  const auto idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_pixels) return;

  const auto scratch_val = image_scratch_buf[idx];
  // convert to 0 if scratch_val is MAX_UINT32_T
  image_buf[idx] = scratch_val == 0xFFFFFFFFu ? 0 : static_cast<uint8_t>(scratch_val);
}

/* -------------------- Renderer Functions ------------------------*/
void LidarDepthRendererCuda::init(int height, int width) {
  if (height < 0 || width < 0) return;
  image_height = height;
  image_width = width;
  
  // need to cuda malloc image buffer and image scratch buffer
  init_image_buf();
}

void LidarDepthRendererCuda::render(cv::Mat &result,
                                    const sensor_msgs::CameraInfo camera_info,
                                    const tf2::Transform &to_camera_tf,
                                    const int bloat_factor) {
  // clear scratch buffer
  set_image_buf();

  // render for each point cloud
  const dim3 threads_per_block(256);
  for (const auto &cloud : *cloud_ptr) {
    // get number of thread blocks
    const auto num_points = cloud.second;
    const dim3 num_blocks((num_points / threads_per_block.x) + 1);
    kernel_render<<<num_blocks, threads_per_block>>>(reinterpret_cast<const CudaPoint*>(cloud.first), 
                                                     cloud.second,
                                                     CudaCameraInfo(camera_info),
                                                     CudaTransform(to_camera_tf),
                                                     bloat_factor,
                                                     image_scratch_buf);
    CHECK_GPU(cudaPeekAtLastError());
  }

  // populate final image buffer
  const auto num_pixels = image_width * image_height;
  const dim3 num_blocks(num_pixels / threads_per_block.x);
  kernel_populate_image_buf<<<num_blocks, threads_per_block>>>(image_scratch_buf,
                                                               image_buf,
                                                               num_pixels);
  
  // populate result cv mat
  CHECK_GPU(cudaMemcpy(result.data, image_buf,
                       image_width * image_height * sizeof(uint8_t),
                       cudaMemcpyDeviceToHost));
}

void LidarDepthRendererCuda::set_cloud(const CloudWindowPtr new_cloud_ptr) {
  cloud_ptr = new_cloud_ptr;
}

// CUDA implementation has enum 2
int LidarDepthRendererCuda::query_implementation() {
  return 2;
}

void LidarDepthRendererCuda::init_image_buf() {
  int num_pixels = image_height * image_width;
  // reallocate depth_image and scratch_space on device memory
  cuda_free(image_scratch_buf);
  cuda_free(image_buf);
  image_scratch_buf = reinterpret_cast<uint32_t*>(cuda_malloc(num_pixels * sizeof(uint32_t)));
  image_buf = reinterpret_cast<uint8_t*>(cuda_malloc(num_pixels * sizeof(uint8_t)));
  // reset image_scratch_buf
  set_image_buf();
}

void LidarDepthRendererCuda::set_image_buf() {
  int num_pixels = image_height * image_width;
  CHECK_GPU(cudaMemset(image_scratch_buf, 0xFF, num_pixels * sizeof(uint32_t)));
}

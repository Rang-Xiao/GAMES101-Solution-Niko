# Niko的GAMES101作业记录

## 01 旋转和投影

### Model变换

- **绕z轴进行旋转**

```c++
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	float rad_angle = rotation_angle / 180.0 * MY_PI;

    model << cos(rad_angle), -sin(rad_angle), 0, 0,
             sin(rad_angle),  cos(rad_angle), 0, 0,
             0,               0,              1, 0,
		     0,               0,              0, 1;

    return model;
}
```

- **绕过原点的任意轴旋转，使用罗德里格斯旋转公式**

```c++
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    float rad_angle = angle / 180.0 * MY_PI;
    float c = cos(rad_angle);
    float s = sin(rad_angle);
    rotation << c + axis.x() * axis.x() * (1 - c), axis.x() * axis.y() * (1 - c) - axis.z() * s, axis.x() * axis.z() * (1 - c) + axis.y() * s, 0,
                axis.y() * axis.x() * (1 - c) + axis.z() * s, c + axis.y() * axis.y() * (1 - c), axis.y() * axis.z() * (1 - c) - axis.x() * s, 0,
                axis.z() * axis.x() * (1 - c) - axis.y() * s, axis.z() * axis.y() * (1 - c) + axis.x() * s, c + axis.z() * axis.z() * (1 - c), 0,
                0, 0, 0, 1;
    return rotation;
}
```

### View变换

```c++
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    // 相当于将相机和物体同步位移，相机移动到原点上，但是只处理了平移
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}
```

### Projetion变换

```c++
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float n = -zNear, f = -zFar;
    // 透视投影->正交投影  挤压
    Eigen::Matrix4f Mpersp_orhtho;
    Mpersp_orhtho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    // 正交投影->正则立方体
    float fovY = eye_fov / 180 * MY_PI;// 角度转弧度
    float t = tan(fovY / 2) * zNear, b = -t;// 
    float r = aspect_ratio * t, l = -r;
    // 转换到正则立方体
    Eigen::Matrix4f Mortho, Mtrans, Mscale;
    Mtrans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Mscale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
		0, 0, 2 / (f - n), 0, // 注意符号，是far - near（负值），实现反向映射。更远的点映射达到1，更近的点映射到-1
        0, 0, 0, 1;
    Mortho = Mscale * Mtrans;
    // 计算得到投影矩阵
    projection = Mortho * Mpersp_orhtho;
    return projection;
}
```

### 结果

为了确保没有轴被错误翻转，选择了其他点来保证图形不是轴对称的

 <img src=".\MDImages\as1.png" alt="as1" style="zoom:50%;" />

## 02 光栅化和Z-buffer

### Z-buffer

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // Find out the bounding box of current triangle.

	float minX = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
	float maxX = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
	float minY = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
	float maxY = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    // iterate through the pixel and find if the current pixel is inside the triangle


    for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                // If so, use the following code to get the interpolated z value.
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha / v[0].w() * v[0].z()  + beta / v[1].w() * v[1].z()  + gamma / v[2].w() * v[2].z();
                z_interpolated *= w_reciprocal;
                
                // 这是经过透视校正的z-buffer，算出了未经过透视变换的w值，从结果上来看更像w-buffer(保存并比较透视变换前的深度信息)
                // 如果不经过校正，直接计算x,y点对应的z插值（透视变换之后的），就是z-buffer

                if (z_interpolated < depth_buf[get_index(x, y)]) {
                    depth_buf[get_index(x, y)] = z_interpolated;
                    // set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
                }
            }
        }
	}
}
```

### 2xSSAA

```c++
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    //--snip--
    if (Enable2xSSAA) {
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                Vector3f final_color(0, 0, 0);
                for (int i = 0; i < 4; ++i) {
                    if (depth_buf_2xSSAA[get_index_2xSSAA(x, y, i)] < std::numeric_limits<float>::infinity()) {
                        // If the subpixel is covered, accumulate its color
                        final_color += frame_buf_2xSSAA[get_index_2xSSAA(x, y, i)] / 4.0f;
                    }
                }
                set_pixel(Eigen::Vector3f(x, y, 1), final_color);
            }
        }
    }
}
```

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // Find out the bounding box of current triangle.

    float minX = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    float maxX = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    float minY = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    float maxY = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
            if (Enable2xSSAA) {
                float subpixel_offsets[4][2] = {
                    {0.25f, 0.25f},
                    {0.75f, 0.25f},
                    {0.25f, 0.75f},
                    {0.75f, 0.75f}
                };
                for (int i = 0; i < 4; ++i) {
                    float sub_x = x + subpixel_offsets[i][0];
                    float sub_y = y + subpixel_offsets[i][1];
                    if (insideTriangle(sub_x, sub_y, t.v)) {
            
                        auto [alpha, beta, gamma] = computeBarycentric2D(sub_x, sub_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha / v[0].w() * v[0].z() + beta / v[1].w() * v[1].z() + gamma / v[2].w() * v[2].z();
                        z_interpolated *= w_reciprocal;

                        if (z_interpolated < depth_buf_2xSSAA[get_index_2xSSAA(x, y, i)]) {
                            depth_buf_2xSSAA[get_index_2xSSAA(x, y, i)] = z_interpolated;
							frame_buf_2xSSAA[get_index_2xSSAA(x, y, i)] = t.getColor();
                        }
                    }
                }
                
            }
            else {
                if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha / v[0].w() * v[0].z() + beta / v[1].w() * v[1].z() + gamma / v[2].w() * v[2].z();
                    z_interpolated *= w_reciprocal;

                    if (z_interpolated < depth_buf[get_index(x, y)]) {
                        depth_buf[get_index(x, y)] = z_interpolated;
                        set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
                    }
                }
            }
            
        }
    }
}
```

### 结果

 <img src=".\MDImages\as2_1.png" alt="as2_1" style="zoom: 33%;" />

<img src=".\MDImages\as2_3.jpeg" alt="as2_3" style="zoom: 80%;" /> <img src=".\MDImages\as2_4.jpeg" alt="as2_4" style="zoom:80%;" />

右图蓝色三角形上侧的白边锯齿改进明显

## 03 着色器

> VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER

### Vertex Shader

```c++
Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}
```

### Normal Fragment Shader

```c++
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}
```

<img src=".\MDImages\normal.png" alt="normal" style="zoom:50%;" />

### Phong Fragment Shader

```c++
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {1000, 1000, 1000}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // Lambertian (Diffuse) Term
		float r = (light.position - point).norm();
		Eigen::Vector3f l = (light.position - point).normalized();
		result_color += kd.cwiseProduct(light.intensity / (r * r)) * std::max(0.f, normal.dot(l));

        // Specular Term
		Eigen::Vector3f v = (eye_pos - point).normalized();
		Eigen::Vector3f h = (l + v).normalized();
        result_color += ks.cwiseProduct(light.intensity / (r * r)) * std::pow(std::max(0.f, normal.dot(h)), p);
    }
    // Ambient Term
    result_color += ka.cwiseProduct(amb_light_intensity);

    return result_color * 255.f;
}
```

<img src=".\MDImages\phong.png" alt="phong" style="zoom:50%;" />

### Texture Fragment Shader

```c++
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
		return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());

    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // Lambertian (Diffuse) Term
        float r = (light.position - point).norm();
        Eigen::Vector3f l = (light.position - point).normalized();
        result_color += kd.cwiseProduct(light.intensity / (r * r)) * std::max(0.f, normal.dot(l));

        // Specular Term
        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (l + v).normalized();
        result_color += ks.cwiseProduct(light.intensity / (r * r)) * std::pow(std::max(0.f, normal.dot(h)), p);
    }
    // Ambient Term
    result_color += ka.cwiseProduct(amb_light_intensity);

    return result_color * 255.f;
}
```



<img src=".\MDImages\texture.png" alt="texture" style="zoom:50%;" />

### Bilinear Interpolation 双线性插值

```c++
Eigen::Vector3f getColor(float u, float v)
{
    u = std::max(0.0f, std::min(1.0f, u));
    v = std::max(0.0f, std::min(1.0f, v));
    auto u_img = u * width;
    auto v_img = (1 - v) * height;
    auto color = image_data.at<cv::Vec3b>(v_img, u_img);
    return Eigen::Vector3f(color[0], color[1], color[2]);
}

Eigen::Vector3f getColorBilinear(float u, float v) 
{
    u = std::max(0.0f, std::min(1.0f, u));
    v = std::max(0.0f, std::min(1.0f, v));
    auto u_img = u * width;
    auto v_img = (1 - v) * height;
    int bl_u = int(u_img);
    int bl_v = int(v_img);
    int tl_u = bl_u;
    int tl_v = bl_v + 1;
    int br_u = bl_u + 1;
    int br_v = bl_v;
    int tr_u = br_u;
	int tr_v = tl_v;
    auto color_bl = image_data.at<cv::Vec3b>(bl_v, bl_u);
    auto color_tl = image_data.at<cv::Vec3b>(tl_v, tl_u);
    auto color_br = image_data.at<cv::Vec3b>(br_v, br_u);
    auto color_tr = image_data.at<cv::Vec3b>(tr_v, tr_u);
    float s = u_img - bl_u;
    float t = v_img - bl_v;
    auto color = (1 - s) * (1 - t) * color_bl + s * (1 - t) * color_br + (1 - s) * t * color_tl + s * t * color_tr;
    return Eigen::Vector3f(color[0], color[1], color[2]);
}
```

<img src=".\MDImages\textureBilinear.png" alt="texture" style="zoom:50%;" />

边界处锯齿得到明显改善（如鼻孔）

### Bump Fragement Shader 凹凸贴图

```c++
Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

	Eigen::Vector3f t = { x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z) };
	Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t, b, normal;

    float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();
	float w = payload.texture->width;
	float h = payload.texture->height;

	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

	Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
	normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}
```

<img src=".\MDImages\bump.png" alt="bump" style="zoom:50%;" />

### Displacement Fragment Shader 位移贴图

```c++
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = { x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z) };
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t, b, normal;

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);

    // 描述点在凹凸变化中的位移
	point += kn * normal * payload.texture->getColor(u, v).norm();

    normal = (TBN * ln).normalized();


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // Lambertian (Diffuse) Term
        float r = (light.position - point).norm();
        Eigen::Vector3f l = (light.position - point).normalized();
        result_color += kd.cwiseProduct(light.intensity / (r * r)) * std::max(0.f, normal.dot(l));

        // Specular Term
        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (l + v).normalized();
        result_color += ks.cwiseProduct(light.intensity / (r * r)) * std::pow(std::max(0.f, normal.dot(h)), p);
    }
    // Ambient Term
    result_color += ka.cwiseProduct(amb_light_intensity);

    return result_color * 255.f;
}
```

<img src=".\MDImages\displacement.png" alt="displacement" style="zoom:50%;" />

## 04 贝塞尔曲线

### Antialiasing Bezier Curve

```c++
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) {
                return control_points[0];
    }

    std::vector<cv::Point2f> new_points;
    for (int i = 0; i < control_points.size() - 1; ++i) {
        auto p0 = control_points[i];
        auto p1 = control_points[i + 1];
        auto point = (1 - t) * p0 + t * p1;
        new_points.push_back(point);
    }

    return recursive_bezier(new_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    // Antialiasing

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto resultPoint = recursive_bezier(control_points, t);

        std::vector<cv::Point2f> pixels(4);
        pixels[0].x = std::round(resultPoint.x);
        pixels[0].y = std::round(resultPoint.y);
		pixels[1].x = pixels[0].x - 1;
		pixels[1].y = pixels[0].y;
		pixels[2].x = pixels[0].x;
		pixels[2].y = pixels[0].y - 1;
		pixels[3].x = pixels[0].x - 1;
		pixels[3].y = pixels[0].y - 1;

        for (const auto &pixel : pixels) 
        {
            float d = std::sqrt(std::pow(pixel.x + 0.5 - resultPoint.x, 2) + std::pow(pixel.y + 0.5 - resultPoint.y, 2));
			float ratio = -d / std::sqrt(2.0) + 1.0;
            window.at<cv::Vec3b>(pixel.y, pixel.x)[2] = 255 * ratio;
        }


    }

}
```

<img src=".\MDImages\compare.png" alt="compare" style="zoom:50%;" />

## 05 光线追踪

<img src=".\MDImages\outputTrace.png" alt="outputTrace" style="zoom:50%;" />

## 06 BVH加速

<img src=".\MDImages\outputBVH.png" alt="outputBVH" style="zoom:50%;" />

import processing.serial.*;

Serial myPort;
float roll, pitch, yaw;
boolean newData = false;

void setup() {
    size(1000, 800, P3D);
    myPort = new Serial(this, "COM3", 9600);
    myPort.bufferUntil('\n');
    background(255);
}

void draw() {
    background(255);
    
    if (newData) {
        newData = false;
    }
    
    translate(width / 2, height / 2, 0);
    
    // Áp dụng ma trận quay vào không gian thế giới, không gắn với đối tượng
    PMatrix3D rotationMatrix = eulerToMatrix(roll, pitch, yaw);
    applyMatrix(rotationMatrix);
    
    drawRectangularPrism(600, 40 ,200);
    displayAngles();
}

void displayAngles() {
    hint(DISABLE_DEPTH_TEST);
    camera();
    fill(0);
    textSize(16);
    text("Roll: " + nf(roll, 1, 2), -width / 2 + 10, -height / 2 + 30);
    text("Pitch: " + nf(pitch, 1, 2), -width / 2 + 10, -height / 2 + 50);
    text("Yaw: " + nf(yaw, 1, 2), -width / 2 + 10, -height / 2 + 70);
    hint(ENABLE_DEPTH_TEST);
}

void drawRectangularPrism(float length, float height, float width) {
    // Mặt trên
    pushMatrix();
    fill(255, 0, 0); // Màu đỏ
    beginShape();
    vertex(-length / 2, -height / 2, -width / 2);
    vertex(length / 2, -height / 2, -width / 2);
    vertex(length / 2, -height / 2, width / 2);
    vertex(-length / 2, -height / 2, width / 2);
    endShape(CLOSE);
    popMatrix();

    // Mặt dưới
    pushMatrix();
    fill(0, 255, 0); // Màu xanh lá
    beginShape();
    vertex(-length / 2, height / 2, -width / 2);
    vertex(length / 2, height / 2, -width / 2);
    vertex(length / 2, height / 2, width / 2);
    vertex(-length / 2, height / 2, width / 2);
    endShape(CLOSE);
    popMatrix();

    // Mặt trước
    pushMatrix();
    fill(0, 0, 255); // Màu xanh dương
    beginShape();
    vertex(-length / 2, -height / 2, width / 2);
    vertex(length / 2, -height / 2, width / 2);
    vertex(length / 2, height / 2, width / 2);
    vertex(-length / 2, height / 2, width / 2);
    endShape(CLOSE);
    popMatrix();

    // Mặt sau
    pushMatrix();
    fill(255, 255, 0); // Màu vàng
    beginShape();
    vertex(-length / 2, -height / 2, -width / 2);
    vertex(length / 2, -height / 2, -width / 2);
    vertex(length / 2, height / 2, -width / 2);
    vertex(-length / 2, height / 2, -width / 2);
    endShape(CLOSE);
    popMatrix();

    // Mặt trái
    pushMatrix();
    fill(255, 165, 0); // Màu cam
    beginShape();
    vertex(-length / 2, -height / 2, -width / 2);
    vertex(-length / 2, height / 2, -width / 2);
    vertex(-length / 2, height / 2, width / 2);
    vertex(-length / 2, -height / 2, width / 2);
    endShape(CLOSE);
    popMatrix();

    // Mặt phải
    pushMatrix();
    fill(255, 0, 255); // Màu hồng
    beginShape();
    vertex(length / 2, -height / 2, -width / 2);
    vertex(length / 2, height / 2, -width / 2);
    vertex(length / 2, height / 2, width / 2);
    vertex(length / 2, -height / 2, width / 2);
    endShape(CLOSE);
    popMatrix();
}

PMatrix3D eulerToMatrix(float roll, float pitch, float yaw) {
    // Chuyển đổi góc Euler thành ma trận quay
    float cr = cos(radians(roll) * 0.5f);
    float cp = cos(radians(pitch) * 0.5f);
    float cy = cos(radians(yaw) * 0.5f);
    float sr = sin(radians(roll) * 0.5f);
    float sp = sin(radians(pitch) * 0.5f);
    float sy = sin(radians(yaw) * 0.5f);
    
    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;
    
    PMatrix3D matrix = new PMatrix3D();
    
    float xx = x * x;
    float xy = x * y;
    float xz = x * z;
    float xw = x * w;
    float yy = y * y;
    float yz = y * z;
    float yw = y * w;
    float zz = z * z;
    float zw = z * w;
    
    matrix.m00 = 1 - 2 * (yy + zz);
    matrix.m01 = 2 * (xy - zw);
    matrix.m02 = 2 * (xz + yw);
    matrix.m03 = 0;
    
    matrix.m10 = 2 * (xy + zw);
    matrix.m11 = 1 - 2 * (xx + zz);
    matrix.m12 = 2 * (yz - xw);
    matrix.m13 = 0;
    
    matrix.m20 = 2 * (xz - yw);
    matrix.m21 = 2 * (yz + xw);
    matrix.m22 = 1 - 2 * (xx + yy);
    matrix.m23 = 0;
    
    matrix.m30 = matrix.m31 = matrix.m32 = 0;
    matrix.m33 = 1;
    
    return matrix;
}

void serialEvent(Serial myPort) {
    String inString = myPort.readStringUntil('\n');
    if (inString != null) {
        inString = trim(inString);
        String[] values = split(inString, ',');
        if (values.length >= 3) {
            roll    = float(values[0]);
            yaw     = float(values[1]);
            pitch   = float(values[2]);
            newData = true;
        }
    }
}

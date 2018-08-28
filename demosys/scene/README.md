# QGLViewer

```c++
/*! Returns the image of \p v by the Quaternion inverse() rotation.
rotate() performs an inverse transformation. Same as inverse().rotate(v). */
Vec Quaternion::inverseRotate(const Vec &v) const {
  return inverse().rotate(v);
}
```

```c++
/*! Computes the modelView matrix associated with the Camera's position() and
 orientation().
 This matrix converts from the world coordinates system to the Camera
 coordinates system, so that coordinates can then be projected on screen using
 the projection matrix (see computeProjectionMatrix()).
 Use getModelViewMatrix() to retrieve this matrix.
 \note You must call this method if your Camera is not associated with a
 QGLViewer and is used for offscreen computations (using
 (un)projectedCoordinatesOf() for instance). loadModelViewMatrix() does it
 otherwise. */
void Camera::computeModelViewMatrix() const {
  if (modelViewMatrixIsUpToDate_)
    return;

  const Quaternion q = frame()->orientation();

  const qreal q00 = 2.0 * q[0] * q[0];
  const qreal q11 = 2.0 * q[1] * q[1];
  const qreal q22 = 2.0 * q[2] * q[2];

  const qreal q01 = 2.0 * q[0] * q[1];
  const qreal q02 = 2.0 * q[0] * q[2];
  const qreal q03 = 2.0 * q[0] * q[3];

  const qreal q12 = 2.0 * q[1] * q[2];
  const qreal q13 = 2.0 * q[1] * q[3];

  const qreal q23 = 2.0 * q[2] * q[3];

  modelViewMatrix_[0] = 1.0 - q11 - q22;
  modelViewMatrix_[1] = q01 - q23;
  modelViewMatrix_[2] = q02 + q13;
  modelViewMatrix_[3] = 0.0;

  modelViewMatrix_[4] = q01 + q23;
  modelViewMatrix_[5] = 1.0 - q22 - q00;
  modelViewMatrix_[6] = q12 - q03;
  modelViewMatrix_[7] = 0.0;

  modelViewMatrix_[8] = q02 - q13;
  modelViewMatrix_[9] = q12 + q03;
  modelViewMatrix_[10] = 1.0 - q11 - q00;
  modelViewMatrix_[11] = 0.0;

  const Vec t = q.inverseRotate(frame()->position());

  modelViewMatrix_[12] = -t.x;
  modelViewMatrix_[13] = -t.y;
  modelViewMatrix_[14] = -t.z;
  modelViewMatrix_[15] = 1.0;

  modelViewMatrixIsUpToDate_ = true;
}
```

```c++
/*! Sets the Camera orientation(), so that it looks at point \p target (defined
 in the world coordinate system).
 The Camera position() is not modified. Simply setViewDirection().
 See also setUpVector(), setOrientation(), showEntireScene(), fitSphere() and
 fitBoundingBox(). */
void Camera::lookAt(const Vec &target) {
  setViewDirection(target - position());
}
```

```c++
/*! Rotates the Camera so that its viewDirection() is \p direction (defined in
 the world coordinate system).
 The Camera position() is not modified. The Camera is rotated so that the
 horizon (defined by its upVector()) is preserved. See also lookAt() and
 setUpVector(). */
void Camera::setViewDirection(const Vec &direction) {
  if (direction.squaredNorm() < 1E-10)
    return;

  Vec xAxis = direction ^ upVector();
  if (xAxis.squaredNorm() < 1E-10) {
    // target is aligned with upVector, this means a rotation around X axis
    // X axis is then unchanged, let's keep it !
    xAxis = frame()->inverseTransformOf(Vec(1.0, 0.0, 0.0));
  }

  Quaternion q;
  q.setFromRotatedBasis(xAxis, xAxis ^ direction, -direction);
  frame()->setOrientationWithConstraint(q);
}
```

```c++
/*! Sets the Quaternion from the three rotated vectors of an orthogonal basis.
  The three vectors do not have to be normalized but must be orthogonal and
  direct (X^Y=k*Z, with k>0).
  \code
  Quaternion q;
  q.setFromRotatedBasis(X, Y, Z);
  // Now q.rotate(Vec(1,0,0)) == X and q.inverseRotate(X) == Vec(1,0,0)
  // Same goes for Y and Z with Vec(0,1,0) and Vec(0,0,1).
  \endcode
  See also setFromRotationMatrix() and Quaternion(const Vec&, const Vec&). */
void Quaternion::setFromRotatedBasis(const Vec &X, const Vec &Y, const Vec &Z) {
  qreal m[3][3];
  qreal normX = X.norm();
  qreal normY = Y.norm();
  qreal normZ = Z.norm();

  for (int i = 0; i < 3; ++i) {
    m[i][0] = X[i] / normX;
    m[i][1] = Y[i] / normY;
    m[i][2] = Z[i] / normZ;
  }

  setFromRotationMatrix(m);
}
```

```c++
/*! Set the Quaternion from a (supposedly correct) 3x3 rotation matrix.
  The matrix is expressed in European format: its three \e columns are the
  images by the rotation of the three vectors of an orthogonal basis. Note that
  OpenGL uses a symmetric representation for its matrices.
  setFromRotatedBasis() sets a Quaternion from the three axis of a rotated
  frame. It actually fills the three columns of a matrix with these rotated
  basis vectors and calls this method. */
void Quaternion::setFromRotationMatrix(const qreal m[3][3]) {
  // Compute one plus the trace of the matrix
  const qreal onePlusTrace = 1.0 + m[0][0] + m[1][1] + m[2][2];

  if (onePlusTrace > 1E-5) {
    // Direct computation
    const qreal s = sqrt(onePlusTrace) * 2.0;
    q[0] = (m[2][1] - m[1][2]) / s;
    q[1] = (m[0][2] - m[2][0]) / s;
    q[2] = (m[1][0] - m[0][1]) / s;
    q[3] = 0.25 * s;
  } else {
    // Computation depends on major diagonal term
    if ((m[0][0] > m[1][1]) & (m[0][0] > m[2][2])) {
      const qreal s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
      q[0] = 0.25 * s;
      q[1] = (m[0][1] + m[1][0]) / s;
      q[2] = (m[0][2] + m[2][0]) / s;
      q[3] = (m[1][2] - m[2][1]) / s;
    } else if (m[1][1] > m[2][2]) {
      const qreal s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
      q[0] = (m[0][1] + m[1][0]) / s;
      q[1] = 0.25 * s;
      q[2] = (m[1][2] + m[2][1]) / s;
      q[3] = (m[0][2] - m[2][0]) / s;
    } else {
      const qreal s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
      q[0] = (m[0][2] + m[2][0]) / s;
      q[1] = (m[1][2] + m[2][1]) / s;
      q[2] = 0.25 * s;
      q[3] = (m[0][1] - m[1][0]) / s;
    }
  }
  normalize();
}
```
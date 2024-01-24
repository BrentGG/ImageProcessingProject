# ImageProcessingProject

We use the [Mediapipe](https://developers.google.com/mediapipe/solutions/vision/pose_landmarker) library for pose detection, this pose detection uses the [BlazePose lite model](https://storage.googleapis.com/mediapipe-assets/Model%20Card%20BlazePose%20GHUM%203D.pdf). The turret attempts to rotate and tilt to make the detected person's face appear in the center of its view. It then proceeds to shoot the target.

We first started out with the Haar Cascades for facial detection which are integrated in OpenCV. However this method's performance was too low. Mediapipe pose detection also has a longer range of detection then the Haar Cascades and the Mediapipe face detection.

We also attempted using two cameras to calculate the distance of the target. However, due to the differences between the cameras, and the fact that they are two seperate cameras instead of a stereo camera unit, this method was very unreliable.

https://github.com/BrentGG/ImageProcessingProject/assets/61016553/f7fde830-0e0c-44f1-98b5-6a3f1d6b2106

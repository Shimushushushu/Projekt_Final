/*---------------------------Modified Private Lib-----------------------------*/

double ApplyDeadband_Kai(double &tar, double deadband);



double Limit_Kai(double &tar, double limit) {
  if (tar > 1.0) {
    return 1.0;
  }
  if (tar < -1.0) {
    return -1.0;
  }
  return tar;
}
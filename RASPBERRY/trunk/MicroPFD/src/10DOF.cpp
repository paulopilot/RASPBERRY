#include "10DOF.h"

10DOF::10DOF()
{
    //ctor
}

10DOF::~10DOF()
{
    //dtor
}


/**************************************************************************/
/*!
    @brief  Populates the .pitch/.roll fields in the sensors_vec_t struct
            with the right angular data (in degree)
    @param  event         The sensors_event_t variable containing the
                          data from the accelerometer
    @param  orientation   The sensors_vec_t object that will have it's
                          .pitch and .roll fields populated
    @return Returns true if the operation was successful, false if there
            was an error

    @code
    bool error;
    sensors_event_t event;
    sensors_vec_t orientation;
    ...
    lsm303accelGetSensorEvent(&event);
    error = accelGetOrientation(&event, &orientation);
    @endcode
*/
/**************************************************************************/
bool 10DOF::accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  if (event == NULL) return false;
  if (orientation == NULL) return false;

  float t_pitch;
  float t_roll;
  float t_heading;
  float signOfZ = event->acceleration.z >= 0 ? 1.0F : -1.0F;

  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*                                 y                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
  orientation->roll = (float)atan2(event->acceleration.y, sqrt(t_roll)) * 180 / PI;

  /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
  /* pitch is positive and increasing when moving upwards                                     */
  /*                                                                                          */
  /*                                 x                                                        */
  /*            pitch = atan(-----------------)                                               */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
  orientation->pitch = (float)atan2(event->acceleration.x, signOfZ * sqrt(t_pitch)) * 180 / PI;

  return true;
}

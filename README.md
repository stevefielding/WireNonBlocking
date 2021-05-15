# WireNonBlocking
Modified Arduino Wire library to add two non-blocking functions.  
Tested with ATSAM3X and ICM-20602 motion sensor.  
The library can be used just like the old Wire library, but the user has
the option of using the non-blocking versions of requestFrom and endTransmission.  
Two new functions added:  
requestFrom_nb and endTransmission_nb
These are state machines and take multiple calls
to complete. See example.
Note that requestFrom_nb needs to be called at least every 25uS for 400KHz i2c,
and every 100uS for 100KHz i2c. If you fail to call within this time frame then
there may be over-run on i2c reads, and the readSuccess will return false.
You can detect readSuccess==false, and drop the returned values.


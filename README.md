# WireNonBlocking
Modified Arduino Wire library to add two non-blocking functions.  
Tested with ATSAM3X and ICM-20602 motion sensor.  
The library can be used just like the old Wire library, but the user has
the option of using the non-blocking versions of requestFrom and endTransmission.  
Two new functions added:  
requestFrom_nb and endTransmission_nb
These are now state machines and take multiple calls
to complete. See example.


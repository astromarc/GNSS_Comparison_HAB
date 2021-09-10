# GNSS_Comparison_HAB
Code used in mission ServetV that configures two GNSS receivers to enable to get GALILEO and GPS data respectively. However, comparison with another GNSS systems or combination is easy achiavable by tweaking the code.

For the Arduino code, please make sure to use the modified TinyGpsPLUS library to enable only GALILEO NMEA sentences: https://github.com/astromarc/TinyGPSPlus. By default, it only accepts only a combination of GNSS ($GN...) or GPS ($GP...). The modification of the library is done for it to accept NMEA sentences starting with $GA... such as $GAGGA.

It uses a sequential process to configure two GNSS receivers with builtin Software Serial Library, and an increased sample rate from the GNSS modules (up to 5Hz) to have a 1 second difference comparison between both GNSS receivers.

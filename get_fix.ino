String get_fix(String gps) {
  // check for the right beginning tag 
  if (gps.substring(1,6) == "GPRMC") {
    // check for a fix
    if (gps.substring(18,19) == "A") {
      //parse the gps data if there is a fix
      String lat_coords = gps.substring(20,29);
      String north_south = gps.substring(30,31);
      String long_coords = gps.substring(32,42);
      String east_west = gps.substring(43,44);
    
      lat_coords.remove(4,1); // remove decimal places
      long_coords.remove(5,1); // remove decimal places 
      String lat_degrees = lat_coords.substring(0,2);
      String lat_decimaldegs = String((lat_coords.substring(2,8) + "00").toInt()/60);
      String long_degrees = long_coords.substring(0,3);
      String long_decimaldegs = String((long_coords.substring(3,9) + "00").toInt()/60);
    
      String lat_sign = "";
      if (north_south == "S") {
        lat_sign = "-";
      }
    
      String long_sign = "";
      if (east_west == "W") {
        long_sign = "-";
      }
  
      return (lat_sign + lat_degrees + "." + lat_decimaldegs + "," + long_sign + long_degrees + "." + long_decimaldegs);
    }
   else {
    return ("-1");
    }
  }
  else {
    return("-2");
  }
}

 

void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
//  String test = "$GPRMC,152034.000,A,4328.3702,N,08032.4138,W,0.34,167.58,180119,,,D*7E"; // string with fixed data 
//  String test = "$GPRMC,022906.799,V,,,,,0.00,0.00,060180,,,N*4A"; // string with void (unfixed) data
  String test = "$GPCGA,"; // string with the wrong starting tag 
  String bledata = "";

  bledata = get_fix(test);

  if (bledata == "-1") {
    Serial.println("No fix found");
  }
  if (bledata == "-2") {
    Serial.println("Don't have right gps line for parsing");
  }
  if (bledata != "-1" && bledata != "-2") {
    Serial.println(bledata);
  }
}

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

 

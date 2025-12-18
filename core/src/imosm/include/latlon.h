#pragma once
#include <math.h>
#include <numbers>

namespace LatLon {
inline constexpr double PI{M_PI};
inline constexpr double PI2{M_PI * 2.0};
inline constexpr double RAD{M_PI / 180.0};
inline constexpr double DEG{180.0 / M_PI};
inline constexpr double R_EARTH{6371e3};

inline constexpr double dmsToDeg(double d, double m, const double s,
                                 const char nsew) {
  double deg{d + m / 60.0 + s / 3600.0};
  if (nsew == 'S' || nsew == 'W') {
    deg = -deg;
  }
  return deg;
}

inline double distance(double lat1, double lon1, double lat2, double lon2) {
  lat1 *= RAD;
  lon1 *= RAD;
  lat2 *= RAD;
  lon2 *= RAD;

  const double dlat{(lat2 - lat1) / 2.0};
  const double dlon{(lon2 - lon1) / 2.0};
  const double a{sin(dlat) * sin(dlat) +
                 cos(lat1) * cos(lat2) * sin(dlon) * sin(dlon)};
  const double c{2.0 * atan2(sqrt(a), sqrt(1.0 - a))};
  const double d{R_EARTH * c};

  return d;
}

inline double bearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 *= RAD;
  lon1 *= RAD;
  lat2 *= RAD;
  lon2 *= RAD;

  const double dlon{lon2 - lon1};
  const double x{cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)};
  const double y{sin(dlon) * cos(lat2)};
  const double th{atan2(y, x)};
  const double b{fmod(th * DEG + 360.0, 360.0)};

  return b;
}

inline void midpoint(double &lat, double &lon, double lat1, double lon1,
                     double lat2, double lon2) {
  lat1 *= RAD;
  lon1 *= RAD;
  lat2 *= RAD;
  lon2 *= RAD;

  const double dlon{lon2 - lon1};
  const double Bx = cos(lat2) * cos(dlon);
  const double By = cos(lat2) * sin(dlon);
  const double y = sin(lat1) + sin(lat2);
  const double x = sqrt((cos(lat1) + Bx) * (cos(lat1) + Bx) + By * By);
  lat = atan2(y, x);
  lon = lon1 + atan2(By, cos(lat1) + Bx);

  lat *= DEG;
  lon *= DEG;
  lon = fmod(lon + 540.0, 360.0) - 180.0;
}

inline void destination(double &lat, double &lon, double lat1, double lon1,
                        double d, double b) {

  lat1 *= RAD;
  lon1 *= RAD;
  b *= RAD;

  const double delta{d / R_EARTH};
  const double sin_lat2{sin(lat1) * cos(delta) +
                        cos(lat1) * sin(delta) * cos(b)};
  const double y{sin(b) * sin(delta) * cos(lat1)};
  const double x{cos(delta) - sin(lat1) * sin_lat2};
  lat = asin(sin_lat2);
  lon = lon1 + atan2(y, x);

  lat *= DEG;
  lon *= DEG;
  lon = fmod((lon + 540.0), 360.0) - 180.0;
}

inline void cartesian(double &x, double &y, double lat, double lon, double lat0,
                      double lon0) {
  lat0 *= RAD;
  lon0 *= RAD;
  lat *= RAD;
  lon *= RAD;

  const double dlon{lon - lon0};
  x = dlon * cos(lat / 2.0 + lat0 / 2.0) * R_EARTH;
  y = (lat - lat0) * R_EARTH;
}
} // namespace LatLon
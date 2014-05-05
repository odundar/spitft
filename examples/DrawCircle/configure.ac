AC_PREREQ(2.61)

# package version number (not shared library version)
# odd micro numbers indicate in-progress development
# even micro numbers indicate released versions
m4_define(DrawCircle_version_major, 0)
m4_define(DrawCircle_version_minor, 1)
m4_define(DrawCircle_version_micro, 0)

m4_define([DrawCircle_version],
          [DrawCircle_version_major.DrawCircle_version_minor.DrawCircle_version_micro])
m4_define([DrawCircle_api_version],
          [DrawCircle_version_major.DrawCircle_version_minor])

AC_INIT(DrawCircle, DrawCircle_version)
AM_INIT_AUTOMAKE(DrawCircle, DrawCircle_version)
AC_CONFIG_HEADERS(config.h)

# Checks for programs.
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_INSTALL

# enable pkg-config
PKG_PROG_PKG_CONFIG

# Checks for library functions.
#AC_CHECK_FUNCS([memset])

AC_SUBST(DrawCircle_CFLAGS)
AC_SUBST(DrawCircle_LIBS)


#icondir=${datadir}/icons/hicolor/32x32/apps
#AC_SUBST(icondir)

AC_OUTPUT([
Makefile
src/Makefile
])

echo ""
echo "   DrawCircle $VERSION"
echo "   ====================="
echo ""
echo "   To build the project, run \"make\""
echo ""

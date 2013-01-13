#!/usr/bin/python
import math
# Open a file
fo = open("headertest.h", "w")
fo.write( '#include "exercise.h"\n' );
fo.write( '#include "sincoslookup.h"\n\n' );
fo.write( 'fixedPoint sinlup(uint16_t deg) {\n' );
fo.write( '\tint8_t factor = 1;\n' );
fo.write( '\tuint8_t quot;\n' );
fo.write( '\tquot = deg / 3600; //fix range\n' );
fo.write( '\tdeg = deg - quot * 3600;\n\n' );
fo.write( '\tquot = deg/900;\n' );
fo.write( '\tdeg = deg - quot * 900;\n' );
fo.write( '\tif (quot == 1) {\n' );
fo.write( '\t\tdeg = 900 - deg;\n' );
fo.write( '\t} else if (quot == 2) {\n' );
fo.write( '\t\tfactor *= -1;\n' );
fo.write( '\t} else if (quot == 3) {\n' );
fo.write( '\t\tfactor *= -1;\n' );
fo.write( '\t\tdeg = 900 - deg;\n' );
fo.write( '\t}\n' );
fo.write( '\tstatic const __flash fixedPoint sinlup[900] = {\n' );
for x in range(0, 900):
	fo.write(str(int(math.sin(math.pi*x/1800)*100)) + ', ');
fo.write( '\t};\n' );
fo.write( '\treturn factor*sinlup[deg];\n' );
fo.write( '}\n\n' );
fo.write( 'fixedPoint coslup(uint16_t deg) {\n' );
fo.write( '\tdeg += 900;\n' );
fo.write( '\treturn sinlup(deg);\n' );
fo.write( '}' );

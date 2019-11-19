rm -f print.zip

#JLC need "Mid X" and "Mid Y" instead of PosX and PosY
find . -type f -name '*pos.csv' -exec sed -i 's/PosX/Mid X/g ; s/PosY/Mid Y/g' {} \;
#One of the bom generation options in kicad is actualy semicolon instead of comma, replace just in case I picked that one
find . -type f -name '*bom.csv' -exec sed -i 's/\;/,/g' {} \;
#These are on the pick and place machine rotated by by some amount, fix here
find . -type f -name '*pos.csv' -exec sed -i '/"Q1",/ s/,0.000000,top/,180.000000,top/;' {} \;
find . -type f -name '*pos.csv' -exec sed -i '/"Q2",/ s/,90.000000,top/,270.000000,top/;' {} \;
find . -type f -name '*pos.csv' -exec sed -i '/"Q3",/ s/,90.000000,top/,270.000000,top/;' {} \;
find . -type f -name '*pos.csv' -exec sed -i '/"U3",/ s/,270.000000,top/,90.000000,top/;' {} \;
find . -type f -name '*pos.csv' -exec sed -i '/"D8",/ s/,270.000000,top/,90.000000,top/;' {} \;
find . -type f -name '*pos.csv' -exec sed -i '/"U2",/ s/,0.000000,top/,270.000000,top/;' {} \;

zip print.zip zero_*
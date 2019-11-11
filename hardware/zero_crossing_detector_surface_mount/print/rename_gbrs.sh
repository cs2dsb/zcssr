rm print.zip

#JLC need "Mid X" and "Mid Y" instead of PosX and PosY
find . -type f -name '*pos.csv' -exec sed -i 's/PosX/Mid X/g ; s/PosY/Mid Y/g' {} \;
#One of the bom generation options in kicad is actualy semicolon instead of comma, replace just in case I picked that one
find . -type f -name '*bom.csv' -exec sed -i 's/\;/,/g' {} \;
#Q1 is on the pick and place machine rotated by 180 degrees, so rotate it here
find . -type f -name '*pos.csv' -exec sed -i '/"Q1",/ s/,0.000000,top/,180.000000,top/;' {} \;

zip print.zip zero_*
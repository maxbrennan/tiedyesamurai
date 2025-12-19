cd ../teaforge-roborio/
echo 'Building libraries...'
./gradlew jar
echo 'Copying libraries...'
cp -- build/libs/* ../Tie-Dye-Samurai/libs/
cd ../Tie-Dye-Samurai/
echo 'Building project...'
./gradlew build
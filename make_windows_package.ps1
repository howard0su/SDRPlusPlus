
if ($args.Count -lt 2) {
	Write-Host "Usage: .\make_windows_package.ps1 <build_dir> <root_dir>"
	exit 1
}

$build_dir = $args[0]
$root_dir = $args[1]

mkdir -Force sdrpp_windows_x64

# Copy root
cp -Force -Recurse $root_dir/*.exe sdrpp_windows_x64/
cp -Force -Recurse $root_dir/*.dll sdrpp_windows_x64/

# Copy core
cp -Force $build_dir/RelWithDebInfo/* sdrpp_windows_x64/
cp -Force 'C:/Program Files/PothosSDR/bin/volk.dll' sdrpp_windows_x64/

# Copy source modules
cp -Force $build_dir/source_modules/audio_source/RelWithDebInfo/audio_source.dll sdrpp_windows_x64/modules/
cp -Force $build_dir/source_modules/file_source/RelWithDebInfo/file_source.dll sdrpp_windows_x64/modules/
cp -Force $build_dir/source_modules/network_source/RelWithDebInfo/network_source.dll sdrpp_windows_x64/modules/
cp -Force $build_dir/source_modules/sddc_source/RelWithDebInfo/sddc_source.dll sdrpp_windows_x64/modules/
cp -Force $build_dir/source_modules/web888_source/RelWithDebInfo/web888_source.dll sdrpp_windows_x64/modules/

# Copy sink modules
cp -Force $build_dir/sink_modules/audio_sink/RelWithDebInfo/audio_sink.dll sdrpp_windows_x64/modules/
cp -Force $build_dir/source_modules/audio_source/RelWithDebInfo/rtaudio.dll sdrpp_windows_x64/

# Copy decoder modules
cp -Force $build_dir/decoder_modules/radio/RelWithDebInfo/radio.dll sdrpp_windows_x64/modules/


# Copy supporting libs
# cp -Force 'C:/Program Files/PothosSDR/bin/pthreadVC2.dll' sdrpp_windows_x64/

del -Force sdrpp_windows_x64.zip
Compress-Archive -Path sdrpp_windows_x64/ -DestinationPath sdrpp_windows_x64.zip

# rm -Force -Recurse sdrpp_windows_x64

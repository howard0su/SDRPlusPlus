
if ($args.Count -lt 2) {
	Write-Host "Usage: .\make_windows_package.ps1 <build_dir> <root_dir>"
	exit 1
}

$build_dir = $args[0]
$root_dir = $args[1]
$build_type = "RelWithDebInfo"
if ($args.Count -ge 3) {
	$build_type = $args[2]
}

function Copy-ModuleDlls($relativePath, $dllName) {
	$srcDir = Join-Path $build_dir $relativePath
	New-Item -ItemType Directory -Force -Path 'sdrpp_windows_x64/modules' | Out-Null

	Copy-Item -Force (Join-Path $srcDir $dllName) 'sdrpp_windows_x64/modules/'

	Get-ChildItem -Path $srcDir -Filter '*.dll' -File |
		Where-Object { $_.Name -ne $dllName } |
		ForEach-Object { Copy-Item -Force $_.FullName 'sdrpp_windows_x64/' }
}

mkdir -Force sdrpp_windows_x64

# Copy root
cp -Force -Recurse $root_dir/* sdrpp_windows_x64/

# Copy core
cp -Force $build_dir/$build_type/*.dll sdrpp_windows_x64/
cp -Force $build_dir/$build_type/*.exe sdrpp_windows_x64/
cp -Force 'C:/Program Files/PothosSDR/bin/volk.dll' sdrpp_windows_x64/

# Copy source modules
Copy-ModuleDlls "source_modules/audio_source/$build_type" 'audio_source.dll'
Copy-ModuleDlls "source_modules/file_source/$build_type" 'file_source.dll'
# Copy-ModuleDlls "source_modules/network_source/$build_type" 'network_source.dll'
Copy-ModuleDlls "source_modules/sddc_source/$build_type" 'sddc_source.dll'
Copy-ModuleDlls "source_modules/web888_source/$build_type" 'web888_source.dll'
# Copy sink modules
Copy-ModuleDlls "sink_modules/audio_sink/$build_type" 'audio_sink.dll'

# Copy decoder modules
# Example usage:
Copy-ModuleDlls "decoder_modules/radio/$build_type" 'radio.dll'
Copy-ModuleDlls "decoder_modules/ft8_decoder/$build_type" 'ft8_decoder.dll'


# Copy supporting libs
# cp -Force 'C:/Program Files/PothosSDR/bin/pthreadVC2.dll' sdrpp_windows_x64/

del -Force sdrpp_windows_x64.zip
Compress-Archive -Path sdrpp_windows_x64/ -DestinationPath sdrpp_windows_x64.zip

# rm -Force -Recurse sdrpp_windows_x64

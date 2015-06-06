ref = -r:M2Mqtt.dll
MonoLidar.exe : MonoLidar.cs Program.cs
	mcs $(ref) ./MonoLidar.cs ./Program.cs

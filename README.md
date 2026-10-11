# Odbiornik stacji meteo Garni 5, v0.3

To jest wstępna wersja programu który odbiera dane z stacji Meteo Garni 5 INT
To jest odpowiednik stacji meteo Bresser.

Program jest napisany w Arduino IDE.
Główna część programu to kopia funkcji z repozytorium "ArturVSS":  
https://github.com/ArturVSS/APRS-WX-station-Garni-2055/tree/main

Ta wersja oprogramowania działa na urządzeniu "Heltec v2" oraz "Heltec v3"
Jestem w trakcie przygotowania tego programu do popularnego modułu z wyświetlaczem 2.8" (ESP32 CYD).
Ostatnio wprowadzono zmiany (dostosowanie do płytki Heltec v3").
Więc trzeba sprawdzić czy tak poprawiony program działa na Heltec v2.

Przy pierwszym uruchomieniu urządzenie ustawia Wi-Fi:
   WIFI_SSID     = "WeatherStation";
   WIFI_PASS     = "meteo123";
Jeśli wcześniej był wgrany i skonfigurowany program to próbuje on połączyć się do zapisanej sieci Wi_Fi
  jeśli nie udaje się to przez 60 sekund to wystawia AccesPoint Wi-Fi jak powyżej
Aby całkowicie skasować ustawienia to w ustawieniach płytki należy wybrać: "Erase All Flash Before Sketch Upload: Enable"

Mam informacje że są problemy z współdziałaniem tego programu z APRS.fi

Projekt nie będzie już rozwijany.
Ale postaram się żeby wszystkie funkcje z tego projektu zostały przeniesione do nowego projektu,
na ESP32 CYD. Nowy projekt nadal będzie miał wszystkie funkcje które powinny pozwolić na uruchomienie go na ESP32 Heltec v2.

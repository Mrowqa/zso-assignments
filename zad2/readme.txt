Rozwiązanie implementuje wszystko z podstawowej wersji sychronicznej,
dodatkowo używa bloku wczytywania poleceń oraz korzysta z algorytmu
oczekiwania na miejsce w kolejce zaproponowanego we wskazówkach.

Zaimplementowałem wstępnie wersję asynchroniczną, ale okazała się
mniej wydajna[1], w dodatku miała deadlocki, a niestety nie było mnie
stać czasowo na debugowanie i ulepszanie mechanizmu, ale z chęcią
posłucham jak to zrobić dobrze.
[1] testowane na demie nagranym w grze, silnik wypisuje FPS-y pod koniec

Rozwiązanie oczywiście testowane pod dłuższym obciążeniem :) .

Synchronizacja:
* mutex na harddooma - używany do dostępu do urządzenia i kolejki:
  - wysyłanie poleceń
  - oczekiwanie na IO na buforach - blokujemy kolejkę
  - blokada urządzenia przy suspend / resume
* mutex na devdooma - do dostępu do kontekstu (aktywnych buforów)
  - setup - atomowo zmieniamy aktywne bufory
  - wysyłanie poleceń:
    * zmiana aktywnych buforów na harddoomie, jeśli jest taka potrzeba
    * trzymamy blokadę do końca wykonywania bloku zadań, by mieć pewność,
      że w międzyczasie któryś setup nie sprawi, że używany bufor zostanie
      zwolniony (aktywne bufory mają zwiększony licznik referencji na file*)
* oczekiwanie na miejsce w kolejce - zaproponowane ze wskazówek, choć
  w praktyce jest ono bez sensu przy synchronicznym blokowaniu urządzenia
  (być może ktoś wyśle bardzo duży bufor poleceń na raz)


Uwaga:
Gdy testowałem suspenda przy pomocy `systemctl suspend`, musiałem odmontować
wszystkie systemy plików korzystające z p9, które nie zostały stworzone
z myślą o uśpieniu (odczyt z takiego mounta zawiesza proces czytający, a OS
nie wyłącza się później w pełni). Przy okazja, alsa również się wysypywała,
ale to już chyba wina silnika gry.

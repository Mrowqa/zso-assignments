Rozwiązanie za 8 pkt.

Nakładać przez git apply na kod jądra 4.20.8.
(Zrobiłem git init z tarballa, potem commit ze zmianami, na końcu wygenerowałem
patcha. U mnie git apply nakłada patcha bez problemu.)

Opis:
- wpięcie się w copy_{to,from}_user
- jest nowe capability
- task_struct dostaje atomic64_t jako bitmaskę, które słowa już wystąpiły

Synchronizacja:
- wypisywanie:
    jest KERN_CONT do printk, ale jak ktoś wypisze coś po drodze,
    to nasz wpis jest rozbijany na kilka;
    zatem mam globalny bufor i na nim spinlocka
- dostęp do bitmaski:
    pierwotnie miałem rwlocka, ale podczas uruchamiania systemu jądro
    wypisywało stack trace, że nie udało się zarejestrować klasy zamka;
    wprawdzie nic nie wybuchało (ten stack trace to nie był crash) i wszystko
    działało, ale nie wiem jak zostałoby to ocenione, więc zamiast tego
    wykorzystałem atomici wraz z operacją fetch_or

    Przy okazji znalazłem:
    https://elixir.bootlin.com/linux/v4.20.8/source/tools/include/linux/spinlock.h#L35
    Heh.


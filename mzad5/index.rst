.. _07-p1-scull:

=================
Przykład 1: scull
=================

Sterownik scull (Simple Character Utility for Loading Localities)
pochodzi z książki "Linux Device Drivers" (A. Rubini, J. Corbet), gdzie 
jego konstrukcja została szczegółowo omówiona (bardzo polecam - 
http://lwn.net/Kernel/LDD3/).

Kod scull implementuje urządzenie (właściwie kilka o tym samym 
działaniu: ``/dev/scull0`` do ``/dev/scull3``) składające się z obszaru pamięci 
(bufora zmiennej długości). Pamięć jest dzielona przez wszystkie 
deskryptory otwartych plików i zachowywana pomiędzy zamknięciem 
urządzenia i ponownym otwarciem. Otwarcie pliku urządzenia w trybie 
tylko do zapisu powoduje usunięcie całej zawartości bufora.

Kod
---

- paczka: :download:`../p1_scull.tar`
- źródło: :download:`access.c`, :download:`main.c`, :download:`pipe.c`, :download:`scull.h`
- Makefile: :download:`Makefile`
- KBUILD: :download:`Kbuild`

Opis implementacji (niestety w języku angielskim)
--------------------------------------------------

The implementation chosen for scull is not a smart one. The source code for a 
smart implementation would be more difficult to read, and the aim of this 
section is to show read and write, not memory management. That's why the code 
just uses ``kmalloc`` and ``kfree`` without resorting to allocation of whole pages, 
although that would be more efficient.

On the flip side, we didn't want to limit the size of the "device" area, for 
both a philosophical reason and a practical one. Philosophically, it's always a 
bad idea to put arbitrary limits on data items being managed. Practically, scull 
can be used to temporarily eat up your system's memory in order to run tests 
under low-memory conditions. Running such tests might help you understand the 
system's internals. You can use the command ``cp /dev/zero /dev/scull0`` to eat all 
the real RAM with scull, and you can use the ``dd`` utility to choose how much data 
is copied to the scull device.

In scull, each device is a linked list of pointers, each of which points to a 
``Scull_Dev`` structure. Each such structure can refer, by default, to at most four 
million bytes, through an array of intermediate pointers. The released source 
uses an array of 1000 pointers to areas of 4000 bytes. We call each memory area 
a quantum and the array (or its length) a quantum set. A scull device and its 
memory areas are shown in Figure 3-1 ("Linux Device Drivers", page 61, available 
at: http://lwn.net/Kernel/LDD3/).

The chosen numbers are such that writing a single byte in scull consumes eight 
or twelve thousand bytes of memory: four thousand for the quantum and four or 
eight thousand for the quantum set (according to whether a pointer is 
represented in 32 bits or 64 bits on the target platform). If, instead, you 
write a huge amount of data, the overhead of the linked list is not too bad. 
There is only one list element for every four megabytes of data, and the maximum 
size of the device is limited by the computer's memory size.

Testy
-----

Bufor przy kopiowaniu::

    cp -f /etc/passwd /dev/scull
    cat /dev/scull

itd...

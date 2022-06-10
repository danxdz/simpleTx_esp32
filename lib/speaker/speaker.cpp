#include "uart.h"
#include <melody_player.h>
#include <melody_factory.h>

int buzzerPin1 = 2;
MelodyPlayer player1(buzzerPin1,2);

void startup () {


  dbout.println("Melody Player - Play melodies simultaneouly");
  dbout.print("Loading melodies... ");
  String notes1[] = { "C4", "G3", "G3", "A3", "G3", "SILENCE", "B3", "C4" };
  Melody melody1 = MelodyFactory.load("Nice Melody", 250, notes1, 8);
  int notes2[] = { 500, 1000, 0, 500 };
  Melody melody2 = MelodyFactory.load("Raw frequencies", 400, notes2, 4);
  dbout.println("Done!");

  dbout.print("Start playing... ");
  player1.playAsync(melody2);

}

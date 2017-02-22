//
// Created by tag on 03/01/17.
//

#pragma once

int init_connection( char *port);

char* wait_connection(char *reponse, int longueur);
void close_connection();

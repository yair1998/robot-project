package com.robotproject.server.routing;

/** Coarse type of an obstruction, supplied by perception. It only selects an expiry lifetime. */
public enum ObstructionKind {
    TRANSIENT,       // moving or short-lived: a person crossing, a cart being pushed
    SEMI_PERMANENT,  // placed or parked: a pallet, a cart left standing
    PERMANENT,       // structural or administrative: closed door, construction; cleared only explicitly
    UNKNOWN          // perception could not tell (the default)
}

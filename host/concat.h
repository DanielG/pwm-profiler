/*
 * concat() - allocate memory and safely concatenate strings in portable C
 * (and C++ if you like).
 *
 * This code deals gracefully with potential integer overflows (perhaps when
 * input strings are maliciously long), as well as with input strings changing
 * from under it (perhaps because of misbehavior of another thread).  It does
 * not depend on non-portable functions such as snprintf() and asprintf().
 *
 * Written by Solar Designer <solar at openwall.com> and placed in the
 * public domain.
 *
 * Originally written for and currently maintained as a part of popa3d,
 * a POP3 server:
 *
 *	http://www.openwall.com/popa3d/
 */

#ifndef _CONCAT_H_
#define _CONCAT_H_

char *concat(const char *s1, ...);

#endif /* _CONCAT_H_ */

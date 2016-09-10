/*
 * Copyright (C) 2005 Arnaldo Carvalho de Melo <acme@ghostprotocols.net>
 * Copyright (C) 2005 Arnaldo Carvalho de Melo <acme@mandriva.com>
 * Copyright (C) 2005 Herbert Xu <herbert@gondor.apana.org.au>
 * Copyright (C) 2005 Ingo Molnar <mingo@elte.hu>
 * Copyright (C) 2005 Linus Torvalds <torvalds@ppc970.osdl.org>
 * Copyright (C) 2005 Paul E. McKenney <paulmck@us.ibm.com>
 * Copyright (C) 2005 Robert Olsson <Robert.Olsson@data.slu.se>
 * Copyright (C) 2006 Akinobu Mita <mita@miraclelinux.com>
 * Copyright (C) 2006 Arnaldo Carvalho de Melo <acme@mandriva.com>
 * Copyright (C) 2006 Dave Jones <davej@redhat.com>
 * Copyright (C) 2006 David Howells <dhowells@redhat.com>
 * Copyright (C) 2006 Oleg Nesterov <oleg@tv-sign.ru>
 * Copyright (C) 2006 Randy Dunlap <rdunlap@xenotime.net>
 * Copyright (C) 2006 Shailabh Nagar <nagar@watson.ibm.com>
 * Copyright (C) 2006 Zach Brown <zach.brown@oracle.com>
 * Copyright (C) 2007 Corey Minyard <minyard@acm.org>
 * Copyright (C) 2007 Daniel Walker <dwalker@mvista.com>
 * Copyright (C) 2007 Denis V. Lunev <den@openvz.org>
 * Copyright (C) 2007 Pavel Emelianov <xemul@sw.ru>
 * Copyright (C) 2007 Pavel Emelyanov <xemul@openvz.org>
 * Copyright (C) 2007 Randy Dunlap <randy.dunlap@oracle.com>
 * Copyright (C) 2007 Robert P. J. Day <rpjday@mindspring.com>
 * Copyright (C) 2008 Franck Bui-Huu <fbuihuu@gmail.com>
 * Copyright (C) 2008 Linus Torvalds <torvalds@linux-foundation.org>
 * Copyright (C) 2008 Luis R. Rodriguez <lrodriguez@atheros.com>
 * Copyright (C) 2008 Masami Hiramatsu <mhiramat@redhat.com>
 * Copyright (C) 2008 Randy Dunlap <randy.dunlap@oracle.com>
 * Copyright (C) 2008 Robert P. J. Day <rpjday@crashcourse.ca>
 * Copyright (C) 2008 Vegard Nossum <vegard.nossum@gmail.com>
 * Copyright (C) 2010 Al Viro <viro@zeniv.linux.org.uk>
 * Copyright (C) 2010 Ben Hutchings <ben@decadent.org.uk>
 * Copyright (C) 2010 Chris Metcalf <cmetcalf@tilera.com>
 * Copyright (C) 2010 David Howells <dhowells@redhat.com>
 * Copyright (C) 2010 Frederic Weisbecker <fweisbec@gmail.com>
 * Copyright (C) 2010 npiggin@suse.de <npiggin@suse.de>
 * Copyright (C) 2011 Linus Torvalds <torvalds@linux-foundation.org>
 * Copyright (C) 2013 Dave Jones <davej@redhat.com>
 * Copyright (C) 2013 Jiri Pirko <jiri@resnulli.us>
 * Copyright (C) 2013 Paul E. McKenney <paulmck@linux.vnet.ibm.com>
 * Copyright (C) 2013 Sasha Levin <sasha.levin@oracle.com>
 * Copyright (C) 2016 Daniel Gröber <dxld@darkboxed.org>
 *
 *   This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef __LIST_H
#define __LIST_H

/* This file is from Linux Kernel (include/linux/list.h)
 * and modified by simply removing hardware prefetching of list items.
 * Here by copyright, credits attributed to wherever they belong.
 * Kulesh Shanmugasundaram (kulesh [squiggly] isis.poly.edu)
 */

/*
 * Simple doubly linked list implementation.
 *
 * Some of the internal functions ("__xxx") are useful when
 * manipulating whole lists rather than single entries, as
 * sometimes we already know the next/prev entries and we can
 * generate better code by using them directly rather than
 * using the generic single-entry routines.
 */

struct list_head {
	struct list_head *next, *prev;
};

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) \
	struct list_head name = LIST_HEAD_INIT(name)

#define INIT_LIST_HEAD(ptr) do { \
	(ptr)->next = (ptr); (ptr)->prev = (ptr); \
} while (0)

/*
 * Insert a new entry between two known consecutive entries.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static inline void __list_add(struct list_head *new,
			      struct list_head *prev,
			      struct list_head *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

/**
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list head to add it after
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 */
static inline void list_add(struct list_head *new, struct list_head *head)
{
	__list_add(new, head, head->next);
}

/**
 * list_add_tail - add a new entry
 * @new: new entry to be added
 * @head: list head to add it before
 *
 * Insert a new entry before the specified head.
 * This is useful for implementing queues.
 */
static inline void list_add_tail(struct list_head *new, struct list_head *head)
{
	__list_add(new, head->prev, head);
}

/*
 * Delete a list entry by making the prev/next entries
 * point to each other.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static inline void __list_del(struct list_head *prev, struct list_head *next)
{
	next->prev = prev;
	prev->next = next;
}

/**
 * list_del - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: list_empty on entry does not return true after this, the entry is in an undefined state.
 */
static inline void list_del(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	entry->next = (void *) 0;
	entry->prev = (void *) 0;
}

/**
 * list_del_init - deletes entry from list and reinitialize it.
 * @entry: the element to delete from the list.
 */
static inline void list_del_init(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	INIT_LIST_HEAD(entry);
}

/**
 * list_move - delete from one list and add as another's head
 * @list: the entry to move
 * @head: the head that will precede our entry
 */
static inline void list_move(struct list_head *list, struct list_head *head)
{
        __list_del(list->prev, list->next);
        list_add(list, head);
}

/**
 * list_move_tail - delete from one list and add as another's tail
 * @list: the entry to move
 * @head: the head that will follow our entry
 */
static inline void list_move_tail(struct list_head *list,
				  struct list_head *head)
{
        __list_del(list->prev, list->next);
        list_add_tail(list, head);
}

/**
 * list_empty - tests whether a list is empty
 * @head: the list to test.
 */
static inline int list_empty(struct list_head *head)
{
	return head->next == head;
}

static inline void __list_splice(struct list_head *list,
				 struct list_head *head)
{
	struct list_head *first = list->next;
	struct list_head *last = list->prev;
	struct list_head *at = head->next;

	first->prev = head;
	head->next = first;

	last->next = at;
	at->prev = last;
}

/**
 * list_splice - join two lists
 * @list: the new list to add.
 * @head: the place to add it in the first list.
 */
static inline void list_splice(struct list_head *list, struct list_head *head)
{
	if (!list_empty(list))
		__list_splice(list, head);
}

/**
 * list_splice_init - join two lists and reinitialise the emptied list.
 * @list: the new list to add.
 * @head: the place to add it in the first list.
 *
 * The list at @list is reinitialised
 */
static inline void list_splice_init(struct list_head *list,
				    struct list_head *head)
{
	if (!list_empty(list)) {
		__list_splice(list, head);
		INIT_LIST_HEAD(list);
	}
}

/**
 * list_entry - get the struct for this entry
 * @ptr:	the &struct list_head pointer.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_struct within the struct.
 */
#define list_entry(ptr, type, member) \
	((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

/**
 * list_first_entry - get the first element from a list
 * @ptr:	the list head to take the element from.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_head within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define list_first_entry(ptr, type, member) \
    list_entry((ptr)->next, type, member)

/**
 * list_for_each	-	iterate over a list
 * @pos:	the &struct list_head to use as a loop counter.
 * @head:	the head for your list.
 */
#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); \
        	pos = pos->next)
/**
 * list_for_each_prev	-	iterate over a list backwards
 * @pos:	the &struct list_head to use as a loop counter.
 * @head:	the head for your list.
 */
#define list_for_each_prev(pos, head) \
	for (pos = (head)->prev; pos != (head); \
        	pos = pos->prev)

/**
 * list_for_each_safe	-	iterate over a list safe against removal of list entry
 * @pos:	the &struct list_head to use as a loop counter.
 * @n:		another &struct list_head to use as temporary storage
 * @head:	the head for your list.
 */
#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

/**
 * list_for_each_entry	-	iterate over list of given type
 * @pos:	the type * to use as a loop counter.
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define list_for_each_entry(pos, head, member)				\
	for (pos = list_entry((head)->next, typeof(*pos), member);	\
	     &pos->member != (head); 					\
	     pos = list_entry(pos->member.next, typeof(*pos), member))

/**
 * list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:	the type * to use as a loop counter.
 * @n:		another type * to use as temporary storage
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define list_for_each_entry_safe(pos, n, head, member)			\
	for (pos = list_entry((head)->next, typeof(*pos), member),	\
		n = list_entry(pos->member.next, typeof(*pos), member);	\
	     &pos->member != (head); 					\
	     pos = n, n = list_entry(n->member.next, typeof(*n), member))


#endif

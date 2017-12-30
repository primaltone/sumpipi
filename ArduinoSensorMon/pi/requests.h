#ifndef REQUESTS_H
#define REQUESTS_H

#include <pthread.h>     /* pthread functions and data structures     */

/* format of a single request. */
struct request {
	int number;		    /* number of the request                  */
	struct request* next;   /* pointer to next request, NULL if none. */
};

struct request_list {
	struct request* requests;     /* head of linked list of requests. */
	struct request* last_request; /* pointer to last request.         */
	int num_requests;	/* number of pending requests, initially none */
	/* global mutex for our program. assignment initializes it. */
	/* note that we use a RECURSIVE mutex, since a handler      */
	/* thread might try to lock it twice consecutively.         */
	pthread_mutex_t request_mutex;

	/* global condition variable for our program. assignment initializes it. */
	pthread_cond_t  got_request;
	void (*handler)(struct request*);
};

void initRequestList (struct request_list *list, void (*handler)(struct request*));

/*
 * function add_request(): add a request to the requests list
 * algorithm: creates a request structure, adds to the list, and
 *            increases number of pending requests by one.
 * input:     request number, linked list mutex.
 * output:    none.
 */
int add_request(struct request_list *list, int request_num, pthread_mutex_t* p_mutex, pthread_cond_t*  p_cond_var);

/*
 * function get_request(): gets the first pending request from the requests list
 *                         removing it from the list.
 * algorithm: creates a request structure, adds to the list, and
 *            increases number of pending requests by one.
 * input:     request number, linked list mutex.
 * output:    pointer to the removed request, or NULL if none.
 * memory:    the returned request need to be freed by the caller.
 */
struct request* get_request(struct request_list *list, pthread_mutex_t* p_mutex);
#if 0
/* sample callback */
/*
 * function handle_request(): handle a single given request.
 * algorithm: prints a message stating that the given thread handled
 *            the given request.
 * input:     request pointer, id of calling thread.
 * output:    none.
 */
void handle_request(struct request* a_request);
#endif
/*
 * function handle_requests_loop(): infinite loop of requests handling
 * algorithm: forever, if there are requests to handle, take the first
 *            and handle it. Then wait on the given condition variable,
 *            and when it is signaled, re-do the loop.
 *            increases number of pending requests by one.
 * input:     id of thread, for printing purposes.
 * output:    none.
 */
void* handle_requests_loop(void* data);
#endif

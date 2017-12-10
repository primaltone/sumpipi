#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>     /* pthread functions and data structures     */
#include "requests.h"

#define DEBUG
#ifdef DEBUG
// Update the following if needed…
#define debug_printf(ARGS...) printf(ARGS)
#else
#define debug_printf(ARGS...) do {} while (0)
#endif
void initRequestList (struct request_list *list, void (*handler)(struct request*)) {
	pthread_mutexattr_t Attr;

	pthread_mutexattr_init(&Attr);
	pthread_mutexattr_settype(&Attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&list->request_mutex, &Attr);

	pthread_cond_init(&list->got_request, NULL/* &cattr*/);

	list->requests = NULL;     /* head of linked list of requests. */
	list->last_request = NULL; /* pointer to last request.         */
	list->num_requests = 0;	/* number of pending requests, initially none */
	list->handler=handler;
}

/*
 * function add_request(): add a request to the requests list
 * algorithm: creates a request structure, adds to the list, and
 *            increases number of pending requests by one.
 * input:     request number, linked list mutex.
 * output:    none.
 */
void add_request(struct request_list *list, int request_num, pthread_mutex_t* p_mutex, pthread_cond_t*  p_cond_var)
{
	int rc;	                    /* return code of pthreads functions.  */
	struct request* a_request;      /* pointer to newly added request.     */

	/* create structure with new request */
	a_request = (struct request*)malloc(sizeof(struct request));
	if (!a_request) { /* malloc failed?? */
		debug_printf("add_request: out of memory\n");
		exit(1);
	}
	a_request->number = request_num;
	a_request->next = NULL;

	/* lock the mutex, to assure exclusive access to the list */
	rc = pthread_mutex_lock(p_mutex);

	/* add new request to the end of the list, updating list */
	/* pointers as required */
	if (list->num_requests == 0) { /* special case - list is empty */
		list->requests = a_request;
		list->last_request = a_request;
	}
	else {
		list->last_request->next = a_request;
		list->last_request = a_request;
	}

	/* increase total number of pending requests by one. */
	list->num_requests++;

	/* unlock mutex */
	rc = pthread_mutex_unlock(p_mutex);

	/* signal the condition variable - there's a new request to handle */
	rc = pthread_cond_signal(p_cond_var);
}

/*
 * function get_request(): gets the first pending request from the requests list
 *                         removing it from the list.
 * algorithm: creates a request structure, adds to the list, and
 *            increases number of pending requests by one.
 * input:     request number, linked list mutex.
 * output:    pointer to the removed request, or NULL if none.
 * memory:    the returned request need to be freed by the caller.
 */
struct request* get_request(struct request_list *list, pthread_mutex_t* p_mutex)
{
	int rc;	                    /* return code of pthreads functions.  */
	struct request* a_request;      /* pointer to request.                 */

	/* lock the mutex, to assure exclusive access to the list */
	rc = pthread_mutex_lock(p_mutex);

	if (list->num_requests > 0) {
		a_request = list->requests;
		list->requests = a_request->next;
		if (list->requests == NULL) { /* this was the last request on the list */
			list->last_request = NULL;
		}
		/* decrease the total number of pending requests */
		list->num_requests--;
	}
	else { /* requests list is empty */
		a_request = NULL;
	}

	/* unlock mutex */
	rc = pthread_mutex_unlock(p_mutex);

	/* return the request to the caller. */
	return a_request;
}
#if 0
/* sample callback */
/*
 * function handle_request(): handle a single given request.
 * algorithm: prints a message stating that the given thread handled
 *            the given request.
 * input:     request pointer, id of calling thread.
 * output:    none.
 */
void handle_request(struct request* a_request)
{
	if (a_request) {
		printf("handled request %d\n", a_request->number);
	}
}
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
void* handle_requests_loop(void* data)
{
	int rc;	                    /* return code of pthreads functions.  */
	struct request* a_request;      /* pointer to a request.               */
	struct request_list *list= (struct request_list*)data;

	/* lock the mutex, to access the requests list exclusively. */
	rc = pthread_mutex_lock(&list->request_mutex);

	/* do forever.... */
	while (1) {
		if (list->num_requests > 0) { /* a request is pending */
			a_request = get_request(list,&list->request_mutex);
			if (a_request) { /* got a request - handle it and free it */
				if (list->handler)
					list->handler(a_request);
				free(a_request);
			}
		}
		else {
			/* wait for a request to arrive. note the mutex will be */
			/* unlocked here, thus allowing other threads access to */
			/* requests list.                                       */
			rc = pthread_cond_wait(&list->got_request, &list->request_mutex);
			/* and after we return from pthread_cond_wait, the mutex  */
			/* is locked again, so we don't need to lock it ourselves */
		}
	}
}

Array.prototype.qsort = function(first, len, cmp)
{
	if (typeof(first) === 'undefined')
		first = 0;
	if (typeof(len) === 'undefined')
		len = this.length - first;
	if (typeof(cmp) === 'undefined')
		cmp = function(a, b) { return a < b; };

	var left = first, stack = [], pos = 0;

	for (; ; ) {                                           /* outer loop */
		for (; left + 1 < len; len++) {                /* sort left to len-1 */
			var pivot = this[left + Math.floor(Math.random() * (len - left))];  /* pick random pivot */
			stack[pos++] = len;                    /* sort right part later */
			for (var right = left - 1; ; ) { /* inner loop: partitioning */
				while (cmp(this[++right], pivot)) { } /* look for greater element */
				while (cmp(pivot, this[--len])) { } /* look for smaller element */
				if (right >= len)
					break;           /* partition point found? */
				var temp = this[right];
				this[right] = this[len];                  /* the only swap */
				this[len] = temp;
			}                            /* partitioned, continue left part */
		}
		if (pos === 0)
			break;                               /* stack empty? */
		left = len;                             /* left to right is sorted */
		len = stack[--pos];                      /* get next range to sort */
	}

	return this;
};

Array.prototype.collapse = function(predicate, length)
{
	if (typeof(length) === 'undefined')
		length = this.length;

	var l = 0;

	for (var c = 0; c < length; ++c)
	{
		// if we can be collapsed, keep l where it is.
		if (predicate(this[c]))
			continue;

		// this node can't be collapsed; push it back as far as we can.
		if (c === l)
		{
			++l;
			continue; // quick exit if we're already in the right spot
		}

		this[l++] = this[c];
	}

	return l;
};

Array.prototype.lower_bound = function(first, last, val, cmp)
{
	if (typeof (cmp) === 'undefined')
		cmp = function(a, b) { return a < b; };

	var count = last - first;
	while (count > 0)
	{
		var step = Math.floor(count / 2);
		var it = first + step;

		if (cmp(this[it], val))
		{
			first = ++it;
			count -= step + 1;
		}
		else
			count = step;
	}
	return first;
};

Array.prototype.upper_bound = function(first, last, val, cmp)
{
	if (typeof (cmp) === 'undefined')
		cmp = function(a, b) { return a < b; };

	var count = last - first;
	while (count > 0)
	{
		var step = Math.floor(count / 2);
		var it = first + step;

		if (!cmp(val, this[it]))
		{
			first = ++it;
			count -= step + 1;
		}
		else
			count = step;
	}
	return first;
};

Array.prototype.rotate = function(first, n_first, last)
{
	var next = n_first;
	while (first != next)
	{
		//std::iter_swap(first++, next++);
		var ofirst = this[first];
		this[first] = this[next];
		this[next] = ofirst;
		++first;
		++next;

		if (next === last)
			next = n_first;
		else if (first === n_first)
			n_first = next;
	}
};
function b2StackQueue(capacity)
{
	this.m_buffer = new Array(capacity);
	this.m_front = 0;
	this.m_back = 0;
	this.m_end = capacity;
}

b2StackQueue.prototype =
{
	Push: function(item)
	{
		if (this.m_back >= this.m_end)
			return;

		this.m_buffer[this.m_back++] = item;
	},

	Pop: function()
	{
'#if @DEBUG';
		b2Assert(this.m_front < this.m_back);
'#endif';
		this.m_front++;
	},

	Empty: function()
	{
		return this.m_front >= this.m_back;
	},

	Front: function()
	{
		return this.m_buffer[this.m_front];
	}
};
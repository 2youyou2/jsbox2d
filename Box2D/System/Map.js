"use strict";

if (typeof(Map) === 'undefined')
{
	var map = (function()
	{
		var _map_id = 0;
		function Map()
		{
			this.size = 0;

			this.map_id = _map_id++;
			this.map_indices = [];
			this.map_indices_len = 0;
			this.map = [];
		}

		Map.prototype =
		{
			get: function(key)
			{
				var index = key['__map_index_for_' + this.map_id];
				return this.map[this.map_id + '_' + index];
			},

			set: function(key, value)
			{
				var valIndex;

				if (this.map_indices.length)
					valIndex = this.map_indices.pop();
				else
					valIndex = this.map_indices_len++;

				var strKey = this.map_id + '_' + valIndex;
				key['__map_index_for_' + this.map_id] = valIndex;

				return this.map[strKey] = value;
			},

			has: function(key)
			{
				return !!key['__map_index_for_' + this.map_id];
			},

			delete: function(key)
			{
				var index = key['__map_index_for_' + this.map_id];
				var value = this.map[this.map_id + '_' + index];

				this.map_indices.push(index);
				delete key['__map_index_for_' + this.map_id];
				delete this.map[this.map_id + '_' + index];

				return value;
			},

			clear: function()
			{
				for (var x in this.map)
					this.delete(x);

				this.map_indices.clear();
				this.map_indices_len = 0;
				this.map.clear();
			}
		};

		return Map;
	})();

	if (typeof(window) !== 'undefined')
		window['Map'] = map;
}
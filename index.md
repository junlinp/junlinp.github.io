---
layout: default
---

# {{ site.title }}

{{ site.description }}

## Posts

{% for post in site.posts %}
- [{{ post.title }}]({{ post.url }}) - {{ post.date | date: "%B %d, %Y" }}
{% endfor %}


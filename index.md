---
layout: default
title: Home
---

<div class="homepage">
    <section class="hero">
        <h1>{{ site.title }}</h1>
        <p>{{ site.description }}</p>
    </section>

    <section class="posts-section" id="posts">
        <h2>Latest Posts</h2>
        <div class="posts-list">
            {% for post in site.posts %}
            <a href="{{ post.url | relative_url }}" class="post-card">
                <h3 class="post-card-title">{{ post.title }}</h3>
                <div class="post-card-meta">
                    <span class="post-card-date">
                        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                            <circle cx="12" cy="12" r="10"></circle>
                            <polyline points="12 6 12 12 16 14"></polyline>
                        </svg>
                        {{ post.date | date: "%B %d, %Y" }}
                    </span>
                    {% if post.categories %}
                    <div class="post-card-categories">
                        {% for category in post.categories %}
                        <span class="category-badge">{{ category }}</span>
                        {% endfor %}
                    </div>
                    {% endif %}
                </div>
            </a>
            {% endfor %}
        </div>
    </section>
</div>

